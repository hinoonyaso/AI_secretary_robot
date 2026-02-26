#!/usr/bin/env python3
import argparse
import sys
from typing import List, Tuple


def load_templates(path: str) -> List[str]:
    templates = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            templates.append(s)
    if not templates:
        raise RuntimeError("no templates loaded")
    return templates


def build_matcher(model_name: str, templates: List[str], device: str):
    # 1) Prefer sentence-transformers path
    try:
        from sentence_transformers import SentenceTransformer, util

        model = SentenceTransformer(model_name, device=device)
        template_emb = model.encode(
            templates, normalize_embeddings=True, convert_to_tensor=True
        )

        def match(text: str) -> Tuple[str, float]:
            text_emb = model.encode([text], normalize_embeddings=True, convert_to_tensor=True)
            scores = util.cos_sim(text_emb, template_emb)[0]
            idx = int(scores.argmax().item())
            score = float(scores[idx].item())
            return templates[idx], score

        return match
    except Exception:
        pass

    # 2) Fallback: direct HF Transformers inference (KoSimCSE compatible)
    import torch
    import torch.nn.functional as F
    from transformers import AutoModel, AutoTokenizer

    torch_device = "cuda" if device == "cuda" and torch.cuda.is_available() else "cpu"
    tokenizer = AutoTokenizer.from_pretrained(model_name)
    model = AutoModel.from_pretrained(model_name).to(torch_device)
    model.eval()

    def encode_texts(texts: List[str]) -> "torch.Tensor":
        with torch.no_grad():
            batch = tokenizer(
                texts,
                padding=True,
                truncation=True,
                max_length=128,
                return_tensors="pt",
            )
            batch = {k: v.to(torch_device) for k, v in batch.items()}
            outputs = model(**batch)
            hidden = outputs.last_hidden_state  # [B, T, H]
            mask = batch["attention_mask"].unsqueeze(-1).expand(hidden.size()).float()
            summed = torch.sum(hidden * mask, dim=1)
            counts = torch.clamp(mask.sum(dim=1), min=1e-9)
            emb = summed / counts
            emb = F.normalize(emb, p=2, dim=1)
            return emb

    template_emb = encode_texts(templates)

    def match(text: str) -> Tuple[str, float]:
        text_emb = encode_texts([text])  # [1, H]
        scores = torch.mm(text_emb, template_emb.transpose(0, 1)).squeeze(0)
        idx = int(torch.argmax(scores).item())
        score = float(scores[idx].item())
        return templates[idx], score

    return match


def serve(args: argparse.Namespace) -> int:
    try:
        templates = load_templates(args.templates)
        matcher = build_matcher(args.model, templates, args.device)
    except Exception as exc:
        print(f"ERROR\tstartup_failed:{exc}", flush=True)
        return 2

    print(f"READY\t{len(templates)}", flush=True)
    for raw in sys.stdin:
        text = raw.strip()
        if not text:
            print("NO_MATCH\t\t0.0", flush=True)
            continue
        if text == "__EXIT__":
            break
        try:
            best, score = matcher(text)
            tag = "MATCH" if score >= args.threshold else "NO_MATCH"
            print(f"{tag}\t{best}\t{score:.6f}", flush=True)
        except Exception as exc:
            print(f"ERROR\tinfer_failed:{exc}", flush=True)
    return 0


def oneshot(args: argparse.Namespace) -> int:
    templates = load_templates(args.templates)
    matcher = build_matcher(args.model, templates, args.device)
    best, score = matcher(args.text)
    tag = "MATCH" if score >= args.threshold else "NO_MATCH"
    print(f"{tag}\t{best}\t{score:.6f}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--serve", action="store_true")
    parser.add_argument("--text", default="")
    parser.add_argument("--model", default="BM-K/KoSimCSE-roberta")
    parser.add_argument("--templates", required=True)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--threshold", type=float, default=0.55)
    args = parser.parse_args()

    if args.serve:
        return serve(args)
    if not args.text:
        print("ERROR\t--text is required when not using --serve", file=sys.stderr)
        return 1
    return oneshot(args)


if __name__ == "__main__":
    raise SystemExit(main())
