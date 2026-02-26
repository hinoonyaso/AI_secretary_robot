#!/usr/bin/env python3
"""
Optimized Moonshine Korean STT Server
- KV Cache 적용
- torch.no_grad() 사용
- CUDA/GPU 지원
- 효율적인 early stopping
"""

import argparse
import json
import math
import os
import socket
import subprocess
import sys
import time
import warnings
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional, List, Tuple
import urllib.error
import urllib.request
import re

import torch
import torch.nn.functional as F

# Runtime locale/env guard for subprocess/ROS pipeline.
os.environ.setdefault("PYTHONIOENCODING", "utf-8")
os.environ.setdefault("LC_ALL", "C.UTF-8")
os.environ.setdefault("LANG", "C.UTF-8")

# Force UTF-8 console output to avoid mojibake in ROS logs/pipes.
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")

# Korean moonshine model repos
_KO_MODEL_REPOS = {
    "moonshine-tiny-ko": "UsefulSensors/moonshine-tiny-ko",
    "moonshine-base-ko": "UsefulSensors/moonshine-base-ko",
}
_KO_MODEL_DIM = {
    "moonshine-tiny-ko": 288,
    "moonshine-base-ko": 416,
}
_KO_MODEL_HEADS = {
    "moonshine-tiny-ko": 8,
    "moonshine-base-ko": 8,
}
_KO_MODEL_ENC_LAYERS = {
    "moonshine-tiny-ko": 6,
    "moonshine-base-ko": 8,
}
_KO_MODEL_DEC_LAYERS = {
    "moonshine-tiny-ko": 6,
    "moonshine-base-ko": 8,
}

# 전역 캐시
_KO_CACHE = {}
_DEVICE = None
_DTYPE = None


def get_device(preferred: str = "auto") -> torch.device:
    """최적 디바이스 선택"""
    global _DEVICE
    if _DEVICE is not None:
        return _DEVICE
    
    if preferred == "auto":
        if torch.cuda.is_available():
            _DEVICE = torch.device("cuda")
        elif torch.backends.mps.is_available():
            _DEVICE = torch.device("mps")
        else:
            _DEVICE = torch.device("cpu")
    else:
        _DEVICE = torch.device(preferred)
    
    print(f"Using device: {_DEVICE}", file=sys.stderr, flush=True)
    return _DEVICE


def get_dtype(compute_type: str = "float16") -> torch.dtype:
    """최적 데이터 타입 선택"""
    global _DTYPE
    if _DTYPE is not None:
        return _DTYPE
    
    device = get_device()
    
    if compute_type == "float16" and device.type == "cuda":
        _DTYPE = torch.float16
    elif compute_type == "bfloat16" and device.type == "cuda":
        _DTYPE = torch.bfloat16
    else:
        _DTYPE = torch.float32
    
    print(f"Using dtype: {_DTYPE}", file=sys.stderr, flush=True)
    return _DTYPE


def load_ko_model_state(model_name: str, device: Optional[torch.device] = None, 
                        dtype: Optional[torch.dtype] = None):
    """모델 상태 로드 (디바이스/타입 지정)"""
    cache_key = f"{model_name}_{device}_{dtype}"
    if cache_key in _KO_CACHE:
        return _KO_CACHE[cache_key]

    import safetensors.torch as st
    import tokenizers as tok_lib
    from huggingface_hub import hf_hub_download

    device = device or get_device()
    dtype = dtype or get_dtype()

    repo = _KO_MODEL_REPOS[model_name]
    state = {
        "dim": _KO_MODEL_DIM[model_name],
        "n_heads": _KO_MODEL_HEADS[model_name],
        "n_enc": _KO_MODEL_ENC_LAYERS[model_name],
        "n_dec": _KO_MODEL_DEC_LAYERS[model_name],
        "bos_id": 1,
        "eos_id": 2,
        "rope_theta": 10000.0,
        "device": device,
        "dtype": dtype,
    }
    state["head_dim"] = state["dim"] // state["n_heads"]
    state["rot_dim"] = max(state["head_dim"] // 2, 32)

    # 모델 가중치 로드 및 디바이스 이동
    weights_path = hf_hub_download(repo, "model.safetensors")
    tok_path = hf_hub_download(repo, "tokenizer.json")
    
    raw_weights = st.load_file(weights_path)
    state["w"] = {
        k: v.to(device=device, dtype=dtype) 
        for k, v in raw_weights.items()
    }
    state["tokenizer"] = tok_lib.Tokenizer.from_file(tok_path)
    
    _KO_CACHE[cache_key] = state
    print(f"Model {model_name} loaded to {device} with {dtype}", file=sys.stderr, flush=True)
    return state


def sanitize_transcript(text: str) -> str:
    if text is None:
        return ""
    original = str(text).strip()
    s = original.replace("\ufffd", "")
    s = "".join(ch for ch in s if ch.isprintable() or ch in "\n\t")
    s = re.sub(r"\s+", " ", s).strip()
    if not s:
        return original
    return s


class MoonshineDecoder:
    """KV Cache를 사용하는 효율적인 디코더"""
    
    def __init__(self, state: dict):
        self.state = state
        self.w = state["w"]
        self.dim = state["dim"]
        self.n_heads = state["n_heads"]
        self.n_enc = state["n_enc"]
        self.n_dec = state["n_dec"]
        self.head_dim = state["head_dim"]
        self.rot_dim = state["rot_dim"]
        self.device = state["device"]
        self.dtype = state["dtype"]
        self.eos_id = state["eos_id"]
        
        # 사전 계산된 인코더 KV (캐시)
        self.enc_k = None
        self.enc_v = None
        self.enc_done = False
        
    def rope_freqs(self, seq_len: int):
        """RoPE 주파수 사전 계산"""
        inv_freq = 1.0 / (self.state["rope_theta"] ** (
            torch.arange(0, self.rot_dim, 2, device=self.device, dtype=torch.float32) / self.rot_dim
        ))
        pos = torch.arange(seq_len, device=self.device, dtype=torch.float32)
        freqs = torch.outer(pos, inv_freq)
        return torch.cos(freqs), torch.sin(freqs)
    
    def apply_rope(self, x, cos, sin):
        """RoPE 적용"""
        x_rot = x[..., :self.rot_dim]
        x_pass = x[..., self.rot_dim:]
        b, h, t, _ = x_rot.shape
        x_p = x_rot.view(b, h, t, self.rot_dim // 2, 2)
        x_even, x_odd = x_p[..., 0], x_p[..., 1]
        c = cos[:t].unsqueeze(0).unsqueeze(0)
        s = sin[:t].unsqueeze(0).unsqueeze(0)
        new_even = x_even * c - x_odd * s
        new_odd = x_odd * c + x_even * s
        rotated = torch.stack([new_even, new_odd], dim=-1).view(b, h, t, self.rot_dim)
        return torch.cat([rotated, x_pass], dim=-1)
    
    def scaled_dot_attn(self, q, k, v, mask=None):
        """스케일드 닷 프로덕트 어텐션"""
        scale = self.head_dim ** -0.5
        a = torch.matmul(q, k.transpose(-2, -1)) * scale
        if mask is not None:
            a = a.masked_fill(~mask, float("-inf"))
        return torch.matmul(F.softmax(a, dim=-1), v)
    
    def encode(self, audio: torch.Tensor):
        """인코더 (한번만 실행, KV 캐시)"""
        if self.enc_done:
            return
        
        w = self.w
        dim = self.dim
        n_heads = self.n_heads
        n_enc = self.n_enc
        
        # Convolutional frontend
        x = audio.view(1, 1, -1)
        x = F.conv1d(x, w["model.encoder.conv1.weight"], stride=64)
        x = torch.tanh(x)
        x = F.group_norm(x, 1, weight=w["model.encoder.groupnorm.weight"],
                        bias=w["model.encoder.groupnorm.bias"])
        x = F.gelu(F.conv1d(x, w["model.encoder.conv2.weight"], 
                           bias=w["model.encoder.conv2.bias"], stride=3))
        x = F.gelu(F.conv1d(x, w["model.encoder.conv3.weight"],
                           bias=w["model.encoder.conv3.bias"], stride=2))
        x = x.permute(0, 2, 1)  # [1, t_enc, dim]
        
        t_enc = x.shape[1]
        e_cos, e_sin = self.rope_freqs(t_enc)
        
        # 인코더 레이어
        for i in range(n_enc):
            p = f"model.encoder.layers.{i}"
            r = x
            x = F.layer_norm(x, [dim], w[f"{p}.input_layernorm.weight"])
            
            # Self attention
            b, t, _ = x.shape
            q = F.linear(x, w[f"{p}.self_attn.q_proj.weight"]).view(
                b, t, n_heads, self.head_dim).transpose(1, 2)
            k = F.linear(x, w[f"{p}.self_attn.k_proj.weight"]).view(
                b, t, n_heads, self.head_dim).transpose(1, 2)
            v = F.linear(x, w[f"{p}.self_attn.v_proj.weight"]).view(
                b, t, n_heads, self.head_dim).transpose(1, 2)
            q = self.apply_rope(q, e_cos, e_sin)
            k = self.apply_rope(k, e_cos, e_sin)
            o = self.scaled_dot_attn(q, k, v).transpose(1, 2).contiguous().view(b, t, dim)
            x = r + F.linear(o, w[f"{p}.self_attn.o_proj.weight"])
            
            # FFN
            r = x
            x = F.layer_norm(x, [dim], w[f"{p}.post_attention_layernorm.weight"])
            h = F.gelu(F.linear(x, w[f"{p}.mlp.fc1.weight"], w[f"{p}.mlp.fc1.bias"]))
            x = r + F.linear(h, w[f"{p}.mlp.fc2.weight"], w[f"{p}.mlp.fc2.bias"])
        
        enc = F.layer_norm(x, [dim], w["model.encoder.layer_norm.weight"])
        
        # 인코더 KV 사전 계산 (디코더 cross-attention용)
        self.enc_k = [
            F.linear(enc, w[f"model.decoder.layers.{i}.encoder_attn.k_proj.weight"])
            .view(1, -1, n_heads, self.head_dim).transpose(1, 2)
            for i in range(self.n_dec)
        ]
        self.enc_v = [
            F.linear(enc, w[f"model.decoder.layers.{i}.encoder_attn.v_proj.weight"])
            .view(1, -1, n_heads, self.head_dim).transpose(1, 2)
            for i in range(self.n_dec)
        ]
        self.enc_done = True
        
    def decode_step(self, tokens: List[int], past_kvs: Optional[List] = None):
        """
        한 스텝 디코딩 (KV Cache 사용)
        Returns: (next_token, updated_past_kvs)
        """
        w = self.w
        dim = self.dim
        n_dec = self.n_dec
        
        t_dec = len(tokens)
        is_first_step = past_kvs is None
        
        # 임베딩
        x = w["model.decoder.embed_tokens.weight"][tokens].unsqueeze(0)
        
        # RoPE (현재 토큰만 또는 전체)
        d_cos, d_sin = self.rope_freqs(t_dec)
        
        # Causal mask (첫 스텝만, 이후는 causal 없음)
        if is_first_step:
            cm = torch.tril(torch.ones(t_dec, t_dec, device=self.device, dtype=torch.bool))
            cm = cm.unsqueeze(0).unsqueeze(0)
        else:
            cm = None  # 이후 스텝: 새 토큰만 처리
        
        new_past_kvs = [] if is_first_step else past_kvs
        
        for i in range(n_dec):
            p = f"model.decoder.layers.{i}"
            
            if is_first_step:
                # 첫 스텝: 전체 시퀀스 처리
                r = x
                x = F.layer_norm(x, [dim], w[f"{p}.input_layernorm.weight"])
                
                # Self attention (causal)
                b, t, _ = x.shape
                q = F.linear(x, w[f"{p}.self_attn.q_proj.weight"]).view(
                    b, t, self.n_heads, self.head_dim).transpose(1, 2)
                k = F.linear(x, w[f"{p}.self_attn.k_proj.weight"]).view(
                    b, t, self.n_heads, self.head_dim).transpose(1, 2)
                v = F.linear(x, w[f"{p}.self_attn.v_proj.weight"]).view(
                    b, t, self.n_heads, self.head_dim).transpose(1, 2)
                q = self.apply_rope(q, d_cos, d_sin)
                k = self.apply_rope(k, d_cos, d_sin)
                o = self.scaled_dot_attn(q, k, v, cm).transpose(1, 2).contiguous().view(b, t, dim)
                x = r + F.linear(o, w[f"{p}.self_attn.o_proj.weight"])
                
                # KV 저장
                self_k = k
                self_v = v
            else:
                # 이후 스텝: 새 토큰만 처리 + 과거 KV 활용
                past_k, past_v = past_kvs[i]
                
                r = x
                x = F.layer_norm(x, [dim], w[f"{p}.input_layernorm.weight"])
                
                # 새 토큰만 계산
                b, t, _ = x.shape
                q = F.linear(x, w[f"{p}.self_attn.q_proj.weight"]).view(
                    b, t, self.n_heads, self.head_dim).transpose(1, 2)
                new_k = F.linear(x, w[f"{p}.self_attn.k_proj.weight"]).view(
                    b, t, self.n_heads, self.head_dim).transpose(1, 2)
                new_v = F.linear(x, w[f"{p}.self_attn.v_proj.weight"]).view(
                    b, t, self.n_heads, self.head_dim).transpose(1, 2)
                
                # RoPE (전체 길이 기준)
                q = self.apply_rope(q, d_cos[-1:], d_sin[-1:])
                new_k = self.apply_rope(new_k, d_cos[-1:], d_sin[-1:])
                
                # KV 캐시 연결
                self_k = torch.cat([past_k, new_k], dim=2)
                self_v = torch.cat([past_v, new_v], dim=2)
                
                # Attention (전체 과거 활용)
                o = self.scaled_dot_attn(q, self_k, self_v).transpose(1, 2).contiguous().view(b, t, dim)
                x = r + F.linear(o, w[f"{p}.self_attn.o_proj.weight"])
            
            # Cross attention (인코더 KV 사용)
            r = x
            x = F.layer_norm(x, [dim], w[f"{p}.post_attention_layernorm.weight"])
            b, t, _ = x.shape
            q_c = F.linear(x, w[f"{p}.encoder_attn.q_proj.weight"]).view(
                b, t, self.n_heads, self.head_dim).transpose(1, 2)
            o_c = self.scaled_dot_attn(q_c, self.enc_k[i], self.enc_v[i])
            o_c = o_c.transpose(1, 2).contiguous().view(b, t, dim)
            x = r + F.linear(o_c, w[f"{p}.encoder_attn.o_proj.weight"])
            
            # FFN (Gated)
            r = x
            x = F.layer_norm(x, [dim], w[f"{p}.final_layernorm.weight"])
            h = F.linear(x, w[f"{p}.mlp.fc1.weight"], w[f"{p}.mlp.fc1.bias"])
            x_up, x_gate = h.chunk(2, dim=-1)
            x = r + F.linear(x_up * F.silu(x_gate), 
                           w[f"{p}.mlp.fc2.weight"], w[f"{p}.mlp.fc2.bias"])
            
            # KV 저장
            new_past_kvs.append((self_k, self_v))
        
        # 출력
        x = F.layer_norm(x, [dim], w["model.decoder.norm.weight"])
        logits = F.linear(x[:, -1, :], w["model.decoder.embed_tokens.weight"])
        next_token = logits.argmax(dim=-1).item()
        
        return next_token, new_past_kvs
    
    def generate(self, max_len: int = 100, early_stop: bool = True) -> str:
        """효율적인 생성 (KV Cache + Early Stopping)"""
        tokens = [self.state["bos_id"]]
        past_kvs = None
        
        for step in range(max_len):
            next_tok, past_kvs = self.decode_step(tokens, past_kvs)
            
            if early_stop and next_tok == self.eos_id:
                break
            tokens.append(next_tok)
            
            # 반복 감지 임계값 축소: 같은 토큰 2회 연속이면 조기 종료
            if len(tokens) >= 2:
                recent = tokens[-2:]
                if len(set(recent)) == 1:
                    break
        
        # 디코딩
        text = self.state["tokenizer"].decode(tokens[1:]).strip()
        
        # 후처리: 반복 제거
        text = self._remove_repetition(text)
        return text
    
    def _remove_repetition(self, text: str) -> str:
        """단순 반복 제거"""
        words = text.split()
        if len(words) < 2:
            return text
        
        # 3회 이상 반복 제거
        result = []
        count = 1
        for i, w in enumerate(words):
            if i > 0 and w == words[i-1]:
                count += 1
                if count <= 2:  # 최대 2회까지만
                    result.append(w)
            else:
                count = 1
                result.append(w)
        
        return " ".join(result)


def transcribe_ko_optimized(audio_path: str, model_name: str,
                          device: Optional[torch.device] = None,
                          dtype: Optional[torch.dtype] = None,
                          max_tokens: int = 100) -> str:
    """최적화된 한국어 음성 인식"""
    import soundfile as sf
    
    # torch.no_grad()로 불필요한 그래디언트 계산 방지
    with torch.no_grad():
        state = load_ko_model_state(model_name, device, dtype)
        
        # 오디오 로드
        audio, sr = sf.read(audio_path, always_2d=False)
        if hasattr(audio, "ndim") and audio.ndim > 1:
            audio = audio.mean(axis=1)
        audio = audio.astype("float32", copy=False)

        # Fast path: 16k input 그대로 사용. 필요 시 경량 리샘플.
        if sr != 16000:
            try:
                from scipy.signal import resample_poly
                g = math.gcd(int(sr), 16000)
                up = 16000 // g
                down = int(sr) // g
                audio = resample_poly(audio, up, down).astype("float32", copy=False)
            except Exception:
                # Fallback only when scipy is unavailable.
                import librosa
                audio = librosa.resample(audio, orig_sr=sr, target_sr=16000)
        audio_tensor = torch.tensor(audio, device=state["device"], 
                                   dtype=state["dtype"])
        
        # 디코더 초기화 및 실행
        decoder = MoonshineDecoder(state)
        
        # 인코딩 (한번)
        start_enc = time.time()
        decoder.encode(audio_tensor)
        enc_time = time.time() - start_enc
        
        # 디코딩 (KV Cache 사용)
        start_dec = time.time()
        # max_len 상향: 의미 있는 문장 길이 보장
        t_enc = audio_tensor.shape[0] // 16000 * 50  # approx tokens
        max_len = min(int(t_enc * 2), max_tokens)
        
        text = decoder.generate(max_len=max_len, early_stop=True)
        dec_time = time.time() - start_dec
        
        print(
            f"Encode: {enc_time:.2f}s, Decode: {dec_time:.2f}s, Total: {enc_time+dec_time:.2f}s",
            file=sys.stderr,
            flush=True,
        )
        return sanitize_transcript(text)


# ============== HTTP 서버 ==============

def is_port_open(host: str, port: int, timeout: float = 0.2) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(timeout)
        return sock.connect_ex((host, port)) == 0


def run_http_server(args) -> int:
    """HTTP 서버 실행"""
    device = get_device(args.device)
    dtype = get_dtype(args.compute_type)
    
    # 모델 사전 로드
    if args.model in _KO_MODEL_REPOS:
        load_ko_model_state(args.model, device, dtype)
        print(f"Model preloaded: {args.model}", file=sys.stderr, flush=True)

    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            if self.path != "/transcribe":
                self.send_response(404)
                self.end_headers()
                return
            
            try:
                length = int(self.headers.get("Content-Length", "0"))
                raw = self.rfile.read(length) if length > 0 else b"{}"
                payload = json.loads(raw.decode("utf-8"))
                
                audio_path = str(payload.get("audio_path", "")).strip()
                model_name = str(payload.get("model", args.model)).strip() or args.model
                
                if not audio_path:
                    raise ValueError("audio_path_empty")
                
                # 최적화된 추론
                start = time.time()
                text = transcribe_ko_optimized(
                    audio_path, model_name, device, dtype, max_tokens=args.max_tokens)
                elapsed = time.time() - start
                
                print(
                    f"Request processed in {elapsed:.2f}s: {text[:50]}...",
                    file=sys.stderr,
                    flush=True,
                )
                
                body = json.dumps({
                    "ok": True, 
                    "text": text,
                    "processing_time_ms": int(elapsed * 1000)
                }).encode("utf-8")
                
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                
            except Exception as exc:
                import traceback
                error_msg = f"{str(exc)}\n{traceback.format_exc()}"
                print(error_msg, file=sys.stderr, flush=True)
                
                body = json.dumps({
                    "ok": False, 
                    "error": str(exc)
                }).encode("utf-8")
                self.send_response(500)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        def log_message(self, format, *args):
            return

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"READY http://{args.host}:{args.port}/transcribe", flush=True)
    
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--audio-path", default="")
    parser.add_argument("--model", default="moonshine-tiny-ko")
    parser.add_argument("--device", default="auto", choices=["auto", "cuda", "cpu"])
    parser.add_argument("--compute-type", default="float16", 
                       choices=["float16", "bfloat16", "float32"])
    parser.add_argument("--max-tokens", type=int, default=100)
    parser.add_argument("--serve-http", action="store_true")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18887)
    args = parser.parse_args()

    warnings.filterwarnings("ignore")

    if args.serve_http:
        return run_http_server(args)
    
    # 단일 실행 모드
    if not args.audio_path:
        print("Usage: --audio-path <path> or --serve-http", file=sys.stderr)
        return 1
    
    device = get_device(args.device)
    dtype = get_dtype(args.compute_type)
    
    with torch.no_grad():
        text = transcribe_ko_optimized(
            args.audio_path, args.model, device, dtype, max_tokens=args.max_tokens)
        print(text)
    
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
