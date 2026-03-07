#!/bin/bash

# Useful when the capture rate of the ToF sensor is too high and you want to use on in every x frames
# Copies the contents of time_stamps.csv, keeping only certain frames. Use with downsample_frames.sh

SRC_CSV="$1"
DST_CSV="$2"
STEP=$3

awk -F',' -v step="$STEP" '
BEGIN { OFS="," }

NR==1 {
    print $0        # keep header
    next
}

((NR-2) % step)==0 {
    $1 = out++      # replace ID with sequential number
    print
}
' "$SRC_CSV" > "$DST_CSV"