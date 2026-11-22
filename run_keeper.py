"""Local entry point: python run_keeper.py demo"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "src" / "goalkeeper_control"))

from goalkeeper_control.cli import main

if __name__ == "__main__":
    raise SystemExit(main(ROOT))
