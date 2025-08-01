#!/bin/bash
SCRIPT_DIR=$(cd $(dirname "${BASH_SOURCE[0]}") && pwd)
python3 $SCRIPT_DIR/create_h5_from_raw.py
python3 $SCRIPT_DIR/cleanup_preprocessing.py
