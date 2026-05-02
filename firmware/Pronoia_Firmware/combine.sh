#!/bin/bash

# Define the output file
OUTPUT_FILE="all_source.txt"

# Clear the output file if it already exists
> "$OUTPUT_FILE"

# Find files and loop through them
# Using IFS= and -r ensures filenames with spaces don't break the script
find src -type f \( -name "*.cpp" -o -name "*.h" \) | while IFS= read -r f; do
  echo "==========================================" >> "$OUTPUT_FILE"
  echo "FILE: $f" >> "$OUTPUT_FILE"
  echo "==========================================" >> "$OUTPUT_FILE"
  cat "$f" >> "$OUTPUT_FILE"
  echo -e "\n" >> "$OUTPUT_FILE"
done

echo "Done! All files combined into $OUTPUT_FILE"