#!/bin/bash
set -e

# GitHub LFS has a 2GB (2147483648 bytes) limit per file
MAX_SIZE=$((2 * 1024 * 1024 * 1024 - 100 * 1024 * 1024))  # 1.9GB to be safe

echo "=================================================="
echo "Split Large Files for Git LFS"
echo "=================================================="
echo ""

if [ -z "$1" ]; then
    echo "ERROR: Directory path required"
    echo "Usage: $0 <directory_path>"
    echo "Example: $0 data/gscuda/map"
    exit 1
fi

TARGET_DIR="$1"

if [ ! -d "$TARGET_DIR" ]; then
    echo "ERROR: Directory not found: $TARGET_DIR"
    exit 1
fi

echo "Scanning directory: $TARGET_DIR"
echo ""

# Find files larger than MAX_SIZE
found_large_files=false

for file in "$TARGET_DIR"/*; do
    if [ -f "$file" ] && [ ! -f "${file}.part.00" ]; then
        size=$(stat -c%s "$file" 2>/dev/null || stat -f%z "$file" 2>/dev/null)
        size_mb=$((size / 1024 / 1024))

        if [ "$size" -gt "$MAX_SIZE" ]; then
            found_large_files=true
            filename=$(basename "$file")
            echo "⚠ Large file found: $filename (${size_mb}MB)"
            echo "  Splitting into 1.9GB chunks..."

            # Split file into 1.9GB chunks
            split -b 1900M "$file" "${file}.part."

            # Rename parts to have proper numbering
            part_num=0
            for part in "${file}.part."*; do
                if [ -f "$part" ]; then
                    mv "$part" "${file}.part.$(printf "%02d" $part_num)"
                    part_size=$(stat -c%s "${file}.part.$(printf "%02d" $part_num)" 2>/dev/null || stat -f%z "${file}.part.$(printf "%02d" $part_num)" 2>/dev/null)
                    part_size_mb=$((part_size / 1024 / 1024))
                    echo "    Created: ${filename}.part.$(printf "%02d" $part_num) (${part_size_mb}MB)"
                    part_num=$((part_num + 1))
                fi
            done

            # Remove original large file
            echo "    Removing original: $filename"
            rm "$file"

            # Create reassembly instructions
            cat > "${file}.REASSEMBLE.txt" << EOF
This file was split into multiple parts due to GitHub's 2GB file size limit.

To reassemble the original file:
  cat ${filename}.part.* > ${filename}

Or use: make reassemble-data DIR=$TARGET_DIR
EOF
            echo "    Created reassembly instructions: ${filename}.REASSEMBLE.txt"
            echo ""
        fi
    fi
done

if [ "$found_large_files" = false ]; then
    echo "✓ No files exceed 2GB limit"
else
    echo "=================================================="
    echo "✓ Large files split successfully!"
    echo "=================================================="
    echo ""
    echo "Next steps:"
    echo "  1. Stage the changes: git add $TARGET_DIR"
    echo "  2. Commit: git commit -m 'Add data with split large files'"
    echo "  3. Push: git push"
fi
