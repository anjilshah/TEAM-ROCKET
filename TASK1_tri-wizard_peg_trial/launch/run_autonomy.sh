#!/usr/bin/env bash
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh" "$@"
exec .venv/bin/python -m peg_sorting.cli autonomy "$@"
