#!/usr/bin/env python3
# 사용법:
# pip install safetensors huggingface_hub tokenizers onnxruntime
# python3 export_moonshine_onnx.py --model moonshine-tiny-ko \
#         --output-dir /home/ubuntu/models/moonshine
#
# 출력 파일:
#   /home/ubuntu/models/moonshine/encoder.onnx
#   /home/ubuntu/models/moonshine/decoder.onnx
#   /home/ubuntu/models/moonshine/vocab.json
#
# 이후 stt_node가 시작될 때 자동으로 이 파일들을 로드한다.
# (onnx_encoder_path, onnx_decoder_path, vocab path 파라미터)

import argparse
import json
import os
import sys
from typing import Dict, Tuple

import numpy as np
import onnxruntime as ort
import safetensors.torch as st
import tokenizers
import torch
import torch.nn as nn
import torch.nn.functional as F
from huggingface_hub import hf_hub_download

MODEL_CONFIGS = {
    "moonshine-tiny-ko": {
        "repo": "UsefulSensors/moonshine-tiny-ko",
        "dim": 288,
        "n_heads": 8,
        "n_enc": 6,
        "n_dec": 6,
        "head_dim": 36,
        "rot_dim": 18,
        "rope_theta": 10000.0,
    }
}


def build_rope_cache(seq_len, rot_dim, rope_theta, device, dtype):
    inv_freq = 1.0 / (
        rope_theta ** (torch.arange(0, rot_dim, 2, device=device, dtype=dtype) / rot_dim)
    )
    pos = torch.arange(seq_len, device=device, dtype=dtype)
    freqs = torch.outer(pos, inv_freq)
    return torch.cos(freqs), torch.sin(freqs)


def apply_rope(x, cos, sin, rot_dim):
    # x: [B, n_heads, T, head_dim]
    # cos,sin: [T, rot_dim//2]
    x_rot = x[..., :rot_dim]
    x_pass = x[..., rot_dim:]
    b, h, t, _ = x_rot.shape
    x_p = x_rot.view(b, h, t, rot_dim // 2, 2)
    x_even = x_p[..., 0]
    x_odd = x_p[..., 1]
    c = cos[:t].unsqueeze(0).unsqueeze(0)
    s = sin[:t].unsqueeze(0).unsqueeze(0)
    new_even = x_even * c - x_odd * s
    new_odd = x_odd * c + x_even * s
    rotated = torch.stack([new_even, new_odd], dim=-1).view(b, h, t, rot_dim)
    return torch.cat([rotated, x_pass], dim=-1)


def scaled_dot_attn(q, k, v, head_dim, mask=None):
    scale = head_dim ** -0.5
    a = torch.matmul(q, k.transpose(-2, -1)) * scale
    if mask is not None:
        a = a.masked_fill(~mask, float("-inf"))
    return torch.matmul(F.softmax(a, dim=-1), v)


class _WeightStore(nn.Module):
    def __init__(self, w: Dict[str, torch.Tensor]):
        super().__init__()
        self._name_map = {}
        for k, v in w.items():
            n = k.replace(".", "_")
            if not torch.is_tensor(v):
                continue
            self.register_buffer(n, v)
            self._name_map[k] = n

    def W(self, key: str) -> torch.Tensor:
        return getattr(self, self._name_map[key])


class MoonshineEncoderModule(_WeightStore):
    def __init__(self, w: Dict[str, torch.Tensor], cfg: dict):
        super().__init__(w)
        self.dim = cfg["dim"]
        self.n_heads = cfg["n_heads"]
        self.n_enc = cfg["n_enc"]
        self.head_dim = cfg["head_dim"]
        self.rot_dim = cfg["rot_dim"]
        self.rope_theta = cfg["rope_theta"]

    def forward(self, audio: torch.Tensor) -> torch.Tensor:
        # audio: [1, num_samples]
        x = audio.view(1, 1, -1)
        x = F.conv1d(x, self.W("model.encoder.conv1.weight"), stride=64)
        x = torch.tanh(x)
        x = F.group_norm(
            x,
            1,
            weight=self.W("model.encoder.groupnorm.weight"),
            bias=self.W("model.encoder.groupnorm.bias"),
        )
        x = F.gelu(
            F.conv1d(
                x,
                self.W("model.encoder.conv2.weight"),
                bias=self.W("model.encoder.conv2.bias"),
                stride=3,
            )
        )
        x = F.gelu(
            F.conv1d(
                x,
                self.W("model.encoder.conv3.weight"),
                bias=self.W("model.encoder.conv3.bias"),
                stride=2,
            )
        )
        x = x.permute(0, 2, 1)  # [1, t_enc, dim]

        t_enc = x.shape[1]
        e_cos, e_sin = build_rope_cache(
            t_enc, self.rot_dim, self.rope_theta, x.device, torch.float32
        )

        for i in range(self.n_enc):
            p = f"model.encoder.layers.{i}"
            r = x
            x = F.layer_norm(x, [self.dim], self.W(f"{p}.input_layernorm.weight"))

            b, t, _ = x.shape
            q = F.linear(x, self.W(f"{p}.self_attn.q_proj.weight")).view(
                b, t, self.n_heads, self.head_dim
            ).transpose(1, 2)
            k = F.linear(x, self.W(f"{p}.self_attn.k_proj.weight")).view(
                b, t, self.n_heads, self.head_dim
            ).transpose(1, 2)
            v = F.linear(x, self.W(f"{p}.self_attn.v_proj.weight")).view(
                b, t, self.n_heads, self.head_dim
            ).transpose(1, 2)

            q = apply_rope(q, e_cos, e_sin, self.rot_dim)
            k = apply_rope(k, e_cos, e_sin, self.rot_dim)

            o = scaled_dot_attn(q, k, v, self.head_dim)
            o = o.transpose(1, 2).contiguous().view(b, t, self.dim)
            x = r + F.linear(o, self.W(f"{p}.self_attn.o_proj.weight"))

            r = x
            x = F.layer_norm(x, [self.dim], self.W(f"{p}.post_attention_layernorm.weight"))
            h = F.gelu(
                F.linear(
                    x,
                    self.W(f"{p}.mlp.fc1.weight"),
                    self.W(f"{p}.mlp.fc1.bias"),
                )
            )
            x = r + F.linear(
                h,
                self.W(f"{p}.mlp.fc2.weight"),
                self.W(f"{p}.mlp.fc2.bias"),
            )

        enc = F.layer_norm(x, [self.dim], self.W("model.encoder.layer_norm.weight"))
        return enc


class MoonshineDecoderModule(_WeightStore):
    def __init__(self, w: Dict[str, torch.Tensor], cfg: dict):
        super().__init__(w)
        self.dim = cfg["dim"]
        self.n_heads = cfg["n_heads"]
        self.n_dec = cfg["n_dec"]
        self.head_dim = cfg["head_dim"]
        self.rot_dim = cfg["rot_dim"]
        self.rope_theta = cfg["rope_theta"]

    def forward(self, input_ids: torch.Tensor, encoder_hidden_states: torch.Tensor) -> torch.Tensor:
        # input_ids: [1, seq_len], encoder_hidden_states: [1, t_enc, dim]
        x = F.embedding(input_ids, self.W("model.decoder.embed_tokens.weight"))
        t_dec = x.shape[1]
        d_cos, d_sin = build_rope_cache(
            t_dec, self.rot_dim, self.rope_theta, x.device, torch.float32
        )

        cm = torch.tril(
            torch.ones((t_dec, t_dec), device=x.device, dtype=torch.bool),
            diagonal=0,
        ).unsqueeze(0).unsqueeze(0)

        for i in range(self.n_dec):
            p = f"model.decoder.layers.{i}"

            r = x
            x = F.layer_norm(x, [self.dim], self.W(f"{p}.input_layernorm.weight"))

            b, t, _ = x.shape
            q = F.linear(x, self.W(f"{p}.self_attn.q_proj.weight")).view(
                b, t, self.n_heads, self.head_dim
            ).transpose(1, 2)
            k = F.linear(x, self.W(f"{p}.self_attn.k_proj.weight")).view(
                b, t, self.n_heads, self.head_dim
            ).transpose(1, 2)
            v = F.linear(x, self.W(f"{p}.self_attn.v_proj.weight")).view(
                b, t, self.n_heads, self.head_dim
            ).transpose(1, 2)

            q = apply_rope(q, d_cos, d_sin, self.rot_dim)
            k = apply_rope(k, d_cos, d_sin, self.rot_dim)

            o = scaled_dot_attn(q, k, v, self.head_dim, cm)
            o = o.transpose(1, 2).contiguous().view(b, t, self.dim)
            x = r + F.linear(o, self.W(f"{p}.self_attn.o_proj.weight"))

            r = x
            x = F.layer_norm(x, [self.dim], self.W(f"{p}.post_attention_layernorm.weight"))

            q_c = F.linear(x, self.W(f"{p}.encoder_attn.q_proj.weight")).view(
                b, t, self.n_heads, self.head_dim
            ).transpose(1, 2)
            k_c = F.linear(encoder_hidden_states, self.W(f"{p}.encoder_attn.k_proj.weight")).view(
                encoder_hidden_states.shape[0],
                encoder_hidden_states.shape[1],
                self.n_heads,
                self.head_dim,
            ).transpose(1, 2)
            v_c = F.linear(encoder_hidden_states, self.W(f"{p}.encoder_attn.v_proj.weight")).view(
                encoder_hidden_states.shape[0],
                encoder_hidden_states.shape[1],
                self.n_heads,
                self.head_dim,
            ).transpose(1, 2)

            o_c = scaled_dot_attn(q_c, k_c, v_c, self.head_dim)
            o_c = o_c.transpose(1, 2).contiguous().view(b, t, self.dim)
            x = r + F.linear(o_c, self.W(f"{p}.encoder_attn.o_proj.weight"))

            r = x
            x = F.layer_norm(x, [self.dim], self.W(f"{p}.final_layernorm.weight"))
            h = F.linear(x, self.W(f"{p}.mlp.fc1.weight"), self.W(f"{p}.mlp.fc1.bias"))
            x_up, x_gate = h.chunk(2, dim=-1)
            x = r + F.linear(
                x_up * F.silu(x_gate),
                self.W(f"{p}.mlp.fc2.weight"),
                self.W(f"{p}.mlp.fc2.bias"),
            )

        x = F.layer_norm(x, [self.dim], self.W("model.decoder.norm.weight"))
        logits = F.linear(x, self.W("model.decoder.embed_tokens.weight"))
        return logits


def extract_vocab_json(tokenizer_json_path: str, output_path: str):
    with open(tokenizer_json_path, "r", encoding="utf-8") as f:
        tok = json.load(f)

    vocab = tok.get("model", {}).get("vocab", {})
    if not vocab:
        vocab = {}
        for item in tok.get("added_tokens", []):
            content = item.get("content")
            idx = item.get("id")
            if content is not None and isinstance(idx, int):
                vocab[content] = idx

    if not vocab:
        raise RuntimeError("tokenizer vocab not found in tokenizer.json")

    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(vocab, f, ensure_ascii=False, indent=2)

    print(f"[moonshine export] Extracting vocab.json ({len(vocab)} tokens)...", flush=True)


def _load_weights_and_tokenizer(model_name: str) -> Tuple[Dict[str, torch.Tensor], str]:
    if model_name not in MODEL_CONFIGS:
        raise ValueError(f"unsupported model: {model_name}")

    repo = MODEL_CONFIGS[model_name]["repo"]
    print("[moonshine export] Loading weights from HuggingFace...", flush=True)

    weights_path = hf_hub_download(repo, "model.safetensors")
    tokenizer_path = hf_hub_download(repo, "tokenizer.json")

    _ = tokenizers.Tokenizer.from_file(tokenizer_path)

    raw = st.load_file(weights_path)
    weights = {}
    for k, v in raw.items():
        if torch.is_floating_point(v):
            weights[k] = v.detach().to(dtype=torch.float32)
        else:
            weights[k] = v.detach()
    return weights, tokenizer_path


def _validate_encoder_onnx(encoder_path: str, audio: torch.Tensor, torch_out: torch.Tensor) -> float:
    sess = ort.InferenceSession(encoder_path, providers=["CPUExecutionProvider"])
    ort_out = sess.run(
        ["encoder_hidden_states"],
        {"input_values": audio.detach().cpu().numpy().astype(np.float32)},
    )[0]
    diff = np.max(np.abs(torch_out.detach().cpu().numpy() - ort_out))
    return float(diff)


def _validate_decoder_onnx(
    decoder_path: str,
    input_ids: torch.Tensor,
    enc_hidden: torch.Tensor,
    torch_logits: torch.Tensor,
) -> float:
    sess = ort.InferenceSession(decoder_path, providers=["CPUExecutionProvider"])
    ort_logits = sess.run(
        ["logits"],
        {
            "input_ids": input_ids.detach().cpu().numpy().astype(np.int64),
            "encoder_hidden_states": enc_hidden.detach().cpu().numpy().astype(np.float32),
        },
    )[0]
    diff = np.max(np.abs(torch_logits.detach().cpu().numpy() - ort_logits))
    return float(diff)


def export_onnx(model_name: str, output_dir: str, device_str: str = "cpu"):
    if model_name not in MODEL_CONFIGS:
        raise ValueError(f"unsupported model: {model_name}")

    cfg = MODEL_CONFIGS[model_name]
    device = torch.device("cpu" if device_str != "cpu" else device_str)

    w, tokenizer_json_path = _load_weights_and_tokenizer(model_name)

    print(
        f"[moonshine export] Model: {model_name} "
        f"(dim={cfg['dim']}, n_enc={cfg['n_enc']}, n_dec={cfg['n_dec']})",
        flush=True,
    )

    encoder = MoonshineEncoderModule(w, cfg).to(device).eval()
    decoder = MoonshineDecoderModule(w, cfg).to(device).eval()

    os.makedirs(output_dir, exist_ok=True)
    encoder_out = os.path.join(output_dir, "encoder.onnx")
    decoder_out = os.path.join(output_dir, "decoder.onnx")
    vocab_out = os.path.join(output_dir, "vocab.json")

    dummy_audio = torch.randn(1, 16000 * 3, dtype=torch.float32, device=device)

    with torch.no_grad():
        enc_hidden = encoder(dummy_audio)
        if enc_hidden.ndim != 3:
            raise RuntimeError(f"encoder output rank invalid: {tuple(enc_hidden.shape)}")

        dummy_input_ids = torch.tensor([[1]], dtype=torch.long, device=device)
        dec_logits = decoder(dummy_input_ids, enc_hidden)
        if dec_logits.ndim != 3:
            raise RuntimeError(f"decoder output rank invalid: {tuple(dec_logits.shape)}")

        print("[moonshine export] Exporting encoder...", flush=True)
        torch.onnx.export(
            encoder,
            (dummy_audio,),
            encoder_out,
            input_names=["input_values"],
            output_names=["encoder_hidden_states"],
            dynamic_axes={
                "input_values": {1: "num_samples"},
                "encoder_hidden_states": {1: "enc_seq_len"},
            },
            opset_version=17,
            do_constant_folding=True,
            training=torch.onnx.TrainingMode.EVAL,
        )

        enc_diff = _validate_encoder_onnx(encoder_out, dummy_audio, enc_hidden)
        enc_status = "PASS" if enc_diff < 1e-3 else "FAIL"
        print(
            f"[moonshine export] Encoder ONNX validation: max_diff={enc_diff:.6f} -> {enc_status}",
            flush=True,
        )
        if enc_diff >= 1e-3:
            raise RuntimeError(f"encoder validation failed: max_diff={enc_diff}")

        print("[moonshine export] Exporting decoder...", flush=True)
        torch.onnx.export(
            decoder,
            (dummy_input_ids, enc_hidden),
            decoder_out,
            input_names=["input_ids", "encoder_hidden_states"],
            output_names=["logits"],
            dynamic_axes={
                "input_ids": {1: "dec_seq_len"},
                "encoder_hidden_states": {1: "enc_seq_len"},
                "logits": {1: "dec_seq_len"},
            },
            opset_version=17,
            do_constant_folding=True,
            training=torch.onnx.TrainingMode.EVAL,
        )

        dec_diff = _validate_decoder_onnx(decoder_out, dummy_input_ids, enc_hidden, dec_logits)
        dec_status = "PASS" if dec_diff < 1e-3 else "FAIL"
        print(
            f"[moonshine export] Decoder ONNX validation: max_diff={dec_diff:.6f} -> {dec_status}",
            flush=True,
        )
        if dec_diff >= 1e-3:
            raise RuntimeError(f"decoder validation failed: max_diff={dec_diff}")

    extract_vocab_json(tokenizer_json_path, vocab_out)
    print("[moonshine export] All done.", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="moonshine-tiny-ko")
    parser.add_argument("--output-dir", default="/home/ubuntu/models/moonshine")
    parser.add_argument(
        "--device",
        default="cpu",
        help="cpu for export (ONNX export은 CPU에서 수행)",
    )
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    export_onnx(args.model, args.output_dir, args.device)

    print("Done. Files saved:")
    print(f"  {args.output_dir}/encoder.onnx")
    print(f"  {args.output_dir}/decoder.onnx")
    print(f"  {args.output_dir}/vocab.json")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"[moonshine export] FAILED: {exc}", file=sys.stderr)
        raise
