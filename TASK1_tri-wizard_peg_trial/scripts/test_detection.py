"""Capture and annotate real rendered RGB-D images (not a pytest module)."""
from peg_sorting.cli import main

if __name__ == "__main__":
    raise SystemExit(main(["vision", "--headless", "--seconds", "1"]))
