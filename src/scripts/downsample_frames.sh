#!/bin/bash

# Useful when the capture rate of the ToF sensor is too high and you want to use on in every x frames
# Copies a sample of files from SRC_DIR to DST_DIR and renames them

SRC_DIR="$1"      # source directory
DST_DIR="$2"  # destination directory
STEP=$3                # keep 1 out of every STEP frames

mkdir -p "$DST_DIR"

i=0
out=0

for f in $(ls "$SRC_DIR"/frame*.ply | sort); do
    if (( i % STEP == 0 )); then
        printf -v newname "frame%05d.ply" "$out"
        cp "$f" "$DST_DIR/$newname"
        ((out++))
    fi
    ((i++))
done