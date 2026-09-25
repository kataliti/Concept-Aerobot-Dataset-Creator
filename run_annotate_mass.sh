#!/usr/bin/env bash
set -euo pipefail
SCRIPT="/home/sed/Desktop/concept-vla/scripts/annotate_mass.py"
if [[ ! -f "$SCRIPT" ]]; then
  echo "Не найден $SCRIPT. Сначала выполните ./install.sh" >&2
  exit 1
fi
exec python3 "$SCRIPT" "$@"
