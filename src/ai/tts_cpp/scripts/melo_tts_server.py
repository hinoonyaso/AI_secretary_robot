#!/usr/bin/env python3
import argparse
import contextlib
import io
import json
import wave
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

    lang = (language or "KR").upper()
    device = resolve_device(requested_device)
    tts = TTS(language=lang, device=device)
    speaker_id = resolve_speaker_id(tts, "", lang)
    return tts, speaker_id


def wav_bytes_to_pcm16_mono(wav_bytes: bytes) -> tuple[bytes, int]:
    with wave.open(io.BytesIO(wav_bytes), "rb") as wf:
        channels = wf.getnchannels()
        sample_rate = wf.getframerate()
        sample_width = wf.getsampwidth()
        frames = wf.readframes(wf.getnframes())

    if channels <= 0:
        raise ValueError("invalid_channel_count")
    if sample_width != 2:
        raise ValueError(f"unsupported_sample_width:{sample_width}")
    if channels == 1:
        return frames, sample_rate

    mono = bytearray()
    frame_size = sample_width * channels
    for i in range(0, len(frames), frame_size):
        frame = frames[i:i + frame_size]
        vals = []
        for ch in range(channels):
            start = ch * sample_width
            vals.append(int.from_bytes(frame[start:start + sample_width], "little", signed=True))
        mixed = int(sum(vals) / channels)
        mixed = max(-32768, min(32767, mixed))
        mono.extend(int(mixed).to_bytes(2, "little", signed=True))
    return bytes(mono), sample_rate


def synthesize_pcm_bytes(tts, speaker_id: int, text: str, speed: float) -> tuple[bytes, int]:
    wav_buffer = io.BytesIO()
    with contextlib.redirect_stdout(io.StringIO()):
        try:
            tts.tts_to_file(text, speaker_id, wav_buffer, speed=speed, format="wav")
        except TypeError:
            tts.tts_to_file(text, speaker_id, wav_buffer, speed=speed)
    wav_buffer.seek(0)
    return wav_bytes_to_pcm16_mono(wav_buffer.read())


class MeloHandler(BaseHTTPRequestHandler):
    tts_model = None
    speaker_id = 0

    def do_GET(self):
        if self.path == "/health":
            body = b"ok"
            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_response(404)
        self.end_headers()

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
            speed = float(payload.get("speed", 1.0))
            if not text:
                raise ValueError("empty_text")

            pcm_bytes, sample_rate = synthesize_pcm_bytes(
                self.tts_model, self.speaker_id, text, speed)
            self.send_response(200)
            self.send_header("Content-Type", "audio/pcm;rate=44100;bits=16;channels=1")
            self.send_header("X-Sample-Rate", str(sample_rate))
            self.send_header("Content-Length", str(len(pcm_bytes)))
            self.end_headers()
            self.wfile.write(pcm_bytes)
        except Exception as exc:
            body = json.dumps({"error": str(exc)}).encode("utf-8")
            self.send_response(500)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    def log_message(self, format, *args):
        return


def main() -> int:
    parser = argparse.ArgumentParser(description="MeloTTS resident PCM server")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5500)
    parser.add_argument("--language", default="KR")
    parser.add_argument("--device", default="auto")
    args = parser.parse_args()

    tts, speaker_id = build_tts(args.language, args.device)
    MeloHandler.tts_model = tts
    MeloHandler.speaker_id = speaker_id

    server = ThreadingHTTPServer((args.host, args.port), MeloHandler)
    print(f"[melo_tts_server] ready on :{args.port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
