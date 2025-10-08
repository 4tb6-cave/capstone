#!/bin/bash

# by chatGPT

# Check if the correct number of arguments are provided
if [ "$#" -ne 3 ]; then
    echo "Usage: $0 <source_dir> <destination_dir> <x>"
    exit 1
fi

# Assign arguments to variables
source_dir="$1"
destination_dir="$2"
x="$3"

# Create destination directory if it doesn't exist
mkdir -p "$destination_dir"

# Initialize counters
total_files=0
copied_files=0

# Check if there are any files matching the pattern
shopt -s nullglob
files=("$source_dir"/*.png)
if [ ${#files[@]} -eq 0 ]; then
    echo "No files found matching the pattern in the source directory."
    exit 1
fi

total_files=${#files[@]}

# Function to display the progress bar
show_progress() {
    local progress=$1
    local total=$2
    local width=50  # Width of the progress bar
    local percent=$((progress * 100 / total))
    local filled=$((percent * width / 100))
    local empty=$((width - filled))

    # Create the progress bar string
    bar=$(printf "%0.s#" $(seq 1 $filled))
    bar+=$(printf "%0.s-" $(seq 1 $empty))

    # Print the progress bar
    printf "\rProgress: [%s] %d%% (%d/%d)" "$bar" "$percent" "$progress" "$total"
}

# Iterate over files in the source directory and copy every x-th file
for ((i = 0; i < total_files; i++)); do
    file="${files[$i]}"

    # Check if the file should be copied
    if (( i % x == 0 )); then
        cp "$file" "$destination_dir"
        copied_files=$((copied_files + 1))
    fi

    # Update the progress bar
    show_progress $((i + 1)) $total_files
done

# Summary of the operation
echo
echo "Total files processed: $total_files"
echo "Total files copied: $copied_files"
echo "Files copied successfully."
