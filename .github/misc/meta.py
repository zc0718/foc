#!/usr/bin/env python3
"""Print one metadata.json field, so shell entry points never re-type a value the file owns."""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def main(argv):
    if len(argv) != 2:
        print('usage: meta.py <field>   e.g. meta.py build_type', file=sys.stderr, flush=True)
        return 2
    try:
        meta = json.loads((ROOT / 'metadata.json').read_text(encoding='utf-8'))
    except (OSError, ValueError) as exc:
        print(f'meta.py: cannot read metadata.json: {exc}', file=sys.stderr, flush=True)
        return 2
    if not isinstance(meta, dict) or argv[1] not in meta:
        print(f'meta.py: metadata.json has no "{argv[1]}"', file=sys.stderr, flush=True)
        return 2
    value = meta[argv[1]]
    print(json.dumps(value) if isinstance(value, (dict, list)) else value, flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
