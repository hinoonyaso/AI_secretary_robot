#!/usr/bin/env python3
import argparse
import contextlib
import json
import io
import os
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


def resolve_device(requested: str) -> str:
    if requested and requested.lower() != "auto":
        return requested
    try:
        import torch
        return "cuda" if torch.cuda.is_available() else "cpu"
    except Exception:
        return "cpu"


def resolve_speaker_id(tts, speaker: str, language: str) -> int:
    spk2id = {}
    try:
        spk2id = dict(getattr(getattr(tts, "hps").data, "spk2id", {}))
    except Exception:
        spk2id = {}

    if speaker:
        if speaker.isdigit():
            return int(speaker)
        if speaker in spk2id:
            return int(spk2id[speaker])
        lower_map = {k.lower(): v for k, v in spk2id.items()}
        if speaker.lower() in lower_map:
            return int(lower_map[speaker.lower()])
        raise ValueError(f"unknown_speaker:{speaker}")

    lang = (language or "").upper()
    if lang in spk2id:
        return int(spk2id[lang])
    if spk2id:
        return int(next(iter(spk2id.values())))
    return 0


def build_tts(language: str, requested_device: str):
    from melo.api import TTS

    device = resolve_device(requested_device)
    lang = (language or "KR").upper()
    tts = TTS(language=lang, device=device)
    return tts, lang


def synthesize_to_file(tts, speaker_id: int, text: str, out_path: str, speed: float) -> None:
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    # Melo internals print progress lines; suppress to keep serve protocol clean.
    with contextlib.redirect_stdout(io.StringIO()):
        tts.tts_to_file(text, speaker_id, out_path, speed=speed)


def serve_loop(args: argparse.Namespace) -> int:
    try:
        tts, language = build_tts(args.language, args.device)
        speaker_id = resolve_speaker_id(tts, args.speaker, language)
    except Exception as exc:
        print(f"ERROR\tstartup_failed:{exc}", flush=True)
        return 2

    out_dir = args.output_dir or "/tmp"
    os.makedirs(out_dir, exist_ok=True)
    print("READY", flush=True)

    seq = 0
    for raw in sys.stdin:
        text = raw.strip()
        if not text:
            print("ERROR\tempty_text", flush=True)
            continue
        if text == "__EXIT__":
            print("BYE", flush=True)
            return 0

        seq += 1
        out_path = os.path.join(out_dir, f"melo_srv_{int(time.time() * 1000)}_{seq}.wav")
        started = time.perf_counter()
        try:
            synthesize_to_file(tts, speaker_id, text, out_path, args.speed)
            elapsed_ms = int((time.perf_counter() - started) * 1000.0)
            print(f"OK\t{out_path}\t{elapsed_ms}", flush=True)
        except Exception as exc:
            print(f"ERROR\tinfer_failed:{exc}", flush=True)

    return 0


def http_server(args: argparse.Namespace) -> int:
    try:
        tts, language = build_tts(args.language, args.device)
        speaker_id = resolve_speaker_id(tts, args.speaker, language)
    except Exception as exc:
        print(f"ERROR startup_failed:{exc}", file=sys.stderr)
        return 2

    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            if self.path != "/synthesize":
                self.send_response(404)
                self.end_headers()
                return

            try:
                length = int(self.headers.get("Content-Length", "0"))
                raw = self.rfile.read(length) if length > 0 else b"{}"
                payload = json.loads(raw.decode("utf-8"))
                text = str(payload.get("text", "")).strip()
                out_path = str(payload.get("output_file", "")).strip()
                speed = float(payload.get("speed", args.speed))
                if not text or not out_path:
                    raise ValueError("missing text/output_file")
                started = time.perf_counter()
                synthesize_to_file(tts, speaker_id, text, out_path, speed)
                elapsed_ms = int((time.perf_counter() - started) * 1000.0)
                body = json.dumps({
                    "ok": True,
                    "output_file": out_path,
                    "elapsed_ms": elapsed_ms,
                }).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            except Exception as exc:
                body = json.dumps({"ok": False, "error": str(exc)}).encode("utf-8")
                self.send_response(500)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        def log_message(self, format, *args):
            return

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"READY http://{args.host}:{args.port}/synthesize", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


def is_port_open(host: str, port: int, timeout: float = 0.2) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(timeout)
        return sock.connect_ex((host, port)) == 0


def autostart_http_server(args: argparse.Namespace) -> None:
    if not args.server_autostart:
        return
    if is_port_open(args.host, args.port):
        return
    cmd = [
        sys.executable,
        os.path.abspath(__file__),
        "--serve-http",
        "--host", args.host,
        "--port", str(args.port),
        "--language", args.language,
        "--speaker", args.speaker,
        "--speed", str(args.speed),
        "--device", args.device,
    ]
    subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        stdin=subprocess.DEVNULL,
        start_new_session=True,
    )
    deadline = time.time() + 30.0
    while time.time() < deadline:
        if is_port_open(args.host, args.port):
            return
        time.sleep(0.1)


def request_http_synthesize(args: argparse.Namespace, text: str, out_path: str) -> bool:
    payload = json.dumps({
        "text": text,
        "output_file": out_path,
        "speed": args.speed,
    }).encode("utf-8")
    req = urllib.request.Request(
        args.server_url,
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=120) as resp:
            body = json.loads(resp.read().decode("utf-8"))
            return bool(body.get("ok")) and os.path.exists(out_path)
    except Exception:
        return False


def main() -> int:
    parser = argparse.ArgumentParser(description="Synthesize speech with MeloTTS")
    parser.add_argument("--text", default="")
    parser.add_argument("--output", default="")
    parser.add_argument("--output_file", default="")
    parser.add_argument("--language", default="KR")
    parser.add_argument("--speaker", default="")
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--serve", action="store_true")
    parser.add_argument("--output_dir", default="/tmp")
    parser.add_argument("--serve-http", action="store_true")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18888)
    parser.add_argument("--server_url", default="http://127.0.0.1:18888/synthesize")
    parser.add_argument("--server_autostart", action="store_true", default=True)
    parser.add_argument("--no_server_autostart", dest="server_autostart", action="store_false")
    args = parser.parse_args()

    if args.serve_http:
        return http_server(args)

    if args.serve:
        return serve_loop(args)

    out_path = args.output or args.output_file
    if not out_path:
        print("missing_output_path", file=sys.stderr)
        return 2
    if not args.text:
        print("missing_text", file=sys.stderr)
        return 2

    if request_http_synthesize(args, args.text, out_path):
        return 0
    autostart_http_server(args)
    if request_http_synthesize(args, args.text, out_path):
        return 0

    try:
        tts, language = build_tts(args.language, args.device)
        speaker_id = resolve_speaker_id(tts, args.speaker, language)
        synthesize_to_file(tts, speaker_id, args.text, out_path, args.speed)
        return 0
    except Exception as exc:
        print(f"melotts_failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
