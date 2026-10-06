#!/usr/bin/env python3
"""Generate the QR wall-marker PNGs for worlds/multi_room.world.

Reads the marker registry (markers/qr_markers.yaml), emits one PNG per
marker next to the registry, and verifies every emitted file round-trips
through cv2.QRCodeDetector().detectAndDecode(). Any marker that does not
decode back to its registered payload fails the run loudly.

Filename convention (derived from the payload, contract in the folder
README): ms05:<room>:<index> -> qr_room_<room>_<index>.png

Usage:
    python3 tools/generate_qr_markers.py            # regenerate + verify
    python3 tools/generate_qr_markers.py --check    # verify existing PNGs only
"""

import argparse
import sys
from pathlib import Path

import cv2
import qrcode
import yaml

MARKERS_DIR = Path(__file__).resolve().parent.parent / 'markers'
DEFAULT_REGISTRY = MARKERS_DIR / 'qr_markers.yaml'


def filename_for(payload: str) -> str:
    prefix, room, index = payload.split(':')
    if prefix != 'ms05':
        raise ValueError(f'payload {payload!r} does not start with "ms05:"')
    return f'qr_room_{room}_{index}.png'


def generate(registry: dict, out_dir: Path) -> list[Path]:
    written = []
    for entry in registry['markers']:
        path = out_dir / filename_for(entry['payload'])
        qr = qrcode.QRCode(
            error_correction=qrcode.constants.ERROR_CORRECT_H,
            box_size=20,
            border=4,  # white quiet zone, in modules
        )
        qr.add_data(entry['payload'])
        qr.make(fit=True)
        qr.make_image(fill_color='black', back_color='white').save(path)
        written.append(path)
        print(f'wrote {path.name}  (payload {entry["payload"]})')
    return written


def verify(registry: dict, out_dir: Path) -> int:
    detector = cv2.QRCodeDetector()
    failures = []
    for entry in registry['markers']:
        path = out_dir / filename_for(entry['payload'])
        if not path.is_file():
            failures.append(f'{path.name}: missing')
            continue
        image = cv2.imread(str(path))
        if image is None:
            failures.append(f'{path.name}: unreadable')
            continue
        data, _, _ = detector.detectAndDecode(image)
        if data != entry['payload']:
            failures.append(
                f'{path.name}: decoded {data!r}, expected {entry["payload"]!r}')
        else:
            print(f'ok    {path.name}  ->  {data}')
    if failures:
        print('\nQR round-trip check FAILED:', file=sys.stderr)
        for line in failures:
            print(f'  {line}', file=sys.stderr)
        return 1
    print(f'\nall {len(registry["markers"])} markers round-trip cleanly')
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--registry', type=Path, default=DEFAULT_REGISTRY,
                        help='marker registry YAML (default: %(default)s)')
    parser.add_argument('--check', action='store_true',
                        help='verify existing PNGs instead of regenerating')
    args = parser.parse_args()

    with open(args.registry) as handle:
        registry = yaml.safe_load(handle)
    out_dir = args.registry.resolve().parent

    if not args.check:
        generate(registry, out_dir)
    return verify(registry, out_dir)


if __name__ == '__main__':
    sys.exit(main())
