#!/usr/bin/env bash
set -euo pipefail

RAW_DIR="/home/ac/datasets/KITTI/raw"
mkdir -p "$RAW_DIR"

cd "$RAW_DIR"
wget -c "https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_10_03_drive_0027/2011_10_03_drive_0027_sync.zip"
wget -c "https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_10_03_drive_0042/2011_10_03_drive_0042_sync.zip"
wget -c "https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_10_03_drive_0034/2011_10_03_drive_0034_sync.zip"
wget -c "https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_10_03_calib.zip"

unzip -o 2011_10_03_drive_0027_sync.zip -d "$RAW_DIR"
unzip -o 2011_10_03_drive_0042_sync.zip -d "$RAW_DIR"
unzip -o 2011_10_03_drive_0034_sync.zip -d "$RAW_DIR"
unzip -o 2011_10_03_calib.zip -d "$RAW_DIR"

echo "Done. raw_dir = $RAW_DIR"
