#!/usr/bin/env bash
# Create .venv and install crs_sim into it (editable) with dev tools.
# Usage: ./setup_venv.sh            (uses python3)
#        PYTHON=python3.11 ./setup_venv.sh
set -euo pipefail
cd "$(dirname "$0")"

PYTHON="${PYTHON:-python3}"
VENV_DIR="${VENV_DIR:-.venv}"

if [ ! -d "$VENV_DIR" ]; then
    echo "Creating $VENV_DIR with $($PYTHON --version)"
    "$PYTHON" -m venv "$VENV_DIR"
fi

# Keep ROS or other globally exported packages out of the venv
unset PYTHONPATH

"$VENV_DIR/bin/python" -m pip install --upgrade pip
"$VENV_DIR/bin/python" -m pip install -e ".[dev]"

echo
echo "Done. Activate with:  source $VENV_DIR/bin/activate"
echo "Interactive sim:      ./start.sh"
