#!/bin/bash

# written by chatGPT

# Check if the correct number of arguments are provided
if [ "$#" -ne 4 ]; then
    echo "Usage: $0 <source_dir> <destination_dir> <start_time> <end_time>"
    exit 1
fi

# Assign arguments to variables
source_dir="$1"
destination_dir="$2"
start_time="$3"
end_time="$4"

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

# Iterate over files in the source directory
for ((i = 0; i < total_files; i++)); do
    file="${files[$i]}"
    
    # Extract the timestamp from the filename
    filename=$(basename -- "$file")
    timestamp=$(echo "$filename" | grep -oP '\d+\.\d+(?=\.png)')
    echo $timestamp
    # Break the loop if the timestamp is greater than the end time
    # if (( $(echo "$timestamp > $end_time" | bc -l) )); then
    #     break
    # fi
    
    # Check if the timestamp is within the specified range using bc for floating point comparison
    if (( $(echo "$timestamp >= $start_time" | bc -l) )) && (( $(echo "$timestamp <= $end_time" | bc -l) )); then
        # Copy the file to the destination directory
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
