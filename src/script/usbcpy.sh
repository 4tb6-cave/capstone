#!/bin/bash

# Copy from bag folder
SOURCE_FOLDER="/etc/cave/bag"

# Define the destination folder name on the USB drive
DESTINATION_FOLDER="bags"

# Get the mount point of the newly plugged-in device
# The mount point is passed as an argument by the udev rule/systemd service
MOUNT_POINT="$1"

# Ensure the mount point exists and is a directory
if [ -d "$MOUNT_POINT" ]; then
    echo "USB drive mounted at $MOUNT_POINT. Starting copy..."

    # Define the full destination path
    FULL_DEST_PATH="$MOUNT_POINT/$DESTINATION_FOLDER"
    
    # Create destination folder if it doesn't exist
    mkdir -p "$FULL_DEST_PATH"

    # Use rsync to copy files (archiving mode, verbose, progress)
    # -a: archive mode (preserves permissions, ownership, timestamps)
    # -v: verbose
    # -r: recursive
    rsync -avr "$SOURCE_FOLDER" "$FULL_DEST_PATH"

    echo "Copy finished. Unmounting device in 5 seconds..."
    sleep 5
    # Unmount the drive safely
    umount "$MOUNT_POINT"

else
    echo "Mount point $MOUNT_POINT not found or not a directory."
fi
