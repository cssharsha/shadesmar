#!/bin/bash
set -e

echo "=================================================="
echo "Reassemble Split Data Files"
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

# Find split files
found_splits=false

for reassemble_file in "$TARGET_DIR"/*.REASSEMBLE.txt; do
    if [ -f "$reassemble_file" ]; then
        found_splits=true

        # Extract original filename
        base_name=$(basename "$reassemble_file" .REASSEMBLE.txt)
        original_file="$TARGET_DIR/$base_name"

        echo "Reassembling: $base_name"

        # Check if all parts exist
        part_count=0
        for part in "${original_file}.part."*; do
            if [ -f "$part" ] && [ "$part" != "$reassemble_file" ]; then
                part_count=$((part_count + 1))
            fi
        done

        if [ "$part_count" -eq 0 ]; then
            echo "  ERROR: No parts found for $base_name"
            continue
        fi

        echo "  Found $part_count parts"

        # Reassemble
        cat "${original_file}.part."* > "$original_file"

        size=$(stat -c%s "$original_file" 2>/dev/null || stat -f%z "$original_file" 2>/dev/null)
        size_mb=$((size / 1024 / 1024))
        echo "  ✓ Reassembled: $base_name (${size_mb}MB)"

        # Optionally remove parts
        echo "  Keep split parts? (y/n)"
        read -t 5 -r keep_parts || keep_parts="y"
        if [ "$keep_parts" = "n" ]; then
            rm "${original_file}.part."*
            rm "$reassemble_file"
            echo "  Removed split parts"
        fi

        echo ""
    fi
done

if [ "$found_splits" = false ]; then
    echo "No split files found in $TARGET_DIR"
else
    echo "=================================================="
    echo "✓ Reassembly complete!"
    echo "=================================================="
fi
