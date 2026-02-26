#!/usr/bin/env python3
import argparse
import asyncio
import os
import pathlib
import site
import sys


def _ensure_edge_tts_importable() -> None:
    try:
        import edge_tts  # noqa: F401
        return
    except Exception:
        pass

    # Some ROS launch environments do not include user-site packages.
    candidates = []
    user_site = site.getusersitepackages()
    if user_site:
        candidates.append(user_site)

    home = os.path.expanduser("~")
    pyver = f"{sys.version_info.major}.{sys.version_info.minor}"
    candidates.append(f"{home}/.local/lib/python{pyver}/site-packages")

    for path in candidates:
        if path and path not in sys.path and os.path.isdir(path):
            sys.path.insert(0, path)

    import edge_tts  # noqa: F401


async def run_tts(text: str, voice: str, rate: str, volume: str, output: str) -> None:
    import edge_tts

    out_path = pathlib.Path(output)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    communicate = edge_tts.Communicate(text=text, voice=voice, rate=rate, volume=volume)
    await communicate.save(output)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--text", required=True)
    parser.add_argument("--voice", default="ko-KR-SunHiNeural")
    parser.add_argument("--rate", default="+0%")
    parser.add_argument("--volume", default="+0%")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    try:
        _ensure_edge_tts_importable()
        asyncio.run(run_tts(args.text, args.voice, args.rate, args.volume, args.output))
    except Exception as exc:
        print(f"edge_tts_failed: {exc}", file=sys.stderr)
        return 3

    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
