#!/usr/bin/env python3
import argparse
import json
import time
from datetime import datetime
from pathlib import Path

from panda import Panda


def log_can(duration_s: float, output_path: Path) -> None:
  panda = Panda()
  start = time.monotonic()
  with output_path.open("w") as f:
    while time.monotonic() - start < duration_s:
      for address, dat, src in panda.can_recv():
        f.write(json.dumps({
          "ts": time.time(),
          "address": address,
          "bytes": dat.hex(),
          "bus": src,
        }) + "\n")
  panda.close()


def main():
  parser = argparse.ArgumentParser(description="Simple Panda CAN logger")
  parser.add_argument("--duration", type=float, default=60.0, help="log duration in seconds")
  parser.add_argument("--output", type=str,
                      default=f"can_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl",
                      help="output file")
  args = parser.parse_args()

  output_path = Path(args.output).resolve()
  print(f"Logging {args.duration}s of CAN traffic to {output_path}")
  log_can(args.duration, output_path)
  print("Done.")


if __name__ == "__main__":
  main()
