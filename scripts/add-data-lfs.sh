#!/bin/bash
set -e

echo "=================================================="
echo "Git LFS Setup and Data Addition"
echo "=================================================="
echo ""

# Check for required arguments
if [ -z "$1" ] || [ -z "$2" ]; then
    echo "ERROR: SOURCE and DEST arguments required"
    echo "Usage: $0 <source_path> <dest_subpath>"
    echo ""
    echo "Examples:"
    echo "  $0 /data/gscudasb/map gscuda/map"
    echo "  $0 /data/gscuda/text gscuda/colmap"
    echo "  $0 /data/mybag.bag datasets/bag1"
    exit 1
fi

SOURCE_PATH="$1"
DEST_SUBPATH="$2"
DEST_PATH="data/$DEST_SUBPATH"

# Check if git LFS is installed
if ! command -v git-lfs &> /dev/null; then
    echo "Git LFS not found. Installing..."
    sudo apt-get update -qq
    sudo apt-get install -y git-lfs
    echo "✓ Git LFS installed"
else
    echo "✓ Git LFS already installed"
fi

# Initialize git LFS in the repo (skip global config if it fails)
echo ""
echo "Initializing Git LFS in repository..."
git lfs install --local || git lfs install --skip-repo || true
echo "✓ Git LFS initialized"

# Configure LFS tracking for common large file types
echo ""
echo "Configuring LFS tracking for large file types..."
git lfs track "*.dat"
git lfs track "*.idx"
git lfs track "*.meta"
git lfs track "*.bag"
git lfs track "*.db3"
git lfs track "*.bin"
echo "✓ LFS tracking configured"

# Show current LFS configuration
echo ""
echo "Current .gitattributes LFS rules:"
grep "filter=lfs" .gitattributes || echo "No LFS rules found yet"

# Check if source exists (file or directory)
if [ ! -e "$SOURCE_PATH" ]; then
    echo ""
    echo "ERROR: Source not found: $SOURCE_PATH"
    exit 1
fi

echo ""
echo "Source: $SOURCE_PATH"
echo "Destination: $DEST_PATH"

# Determine if source is a file or directory
if [ -f "$SOURCE_PATH" ]; then
    # Source is a file
    echo ""
    echo "Source is a file, copying to destination directory..."
    mkdir -p "$DEST_PATH"
    filename=$(basename "$SOURCE_PATH")
    filesize=$(du -h "$SOURCE_PATH" | cut -f1)
    echo "  $filename ($filesize)"
    cp "$SOURCE_PATH" "$DEST_PATH/"
    echo "✓ File copied"
elif [ -d "$SOURCE_PATH" ]; then
    # Source is a directory
    echo ""
    echo "Source is a directory, copying all contents..."
    mkdir -p "$DEST_PATH"

    # Copy all files from source to destination
    for file in "$SOURCE_PATH"/*; do
        if [ -f "$file" ]; then
            filename=$(basename "$file")
            filesize=$(du -h "$file" | cut -f1)
            echo "  $filename ($filesize)"
            cp "$file" "$DEST_PATH/"
        elif [ -d "$file" ]; then
            dirname=$(basename "$file")
            echo "  $dirname/ (directory)"
            cp -r "$file" "$DEST_PATH/"
        fi
    done
    echo "✓ Directory contents copied"
fi

# Stage the .gitattributes changes
echo ""
echo "Staging .gitattributes..."
git add .gitattributes

# Stage the data files
echo ""
echo "Staging data files (this may take a moment for large files)..."
git add "$DEST_PATH"

# Show status
echo ""
echo "Git status:"
git status --short

echo ""
echo "=================================================="
echo "✓ Setup complete!"
echo "=================================================="
echo ""
echo "Next steps:"
echo "  1. Review changes: git status"
echo "  2. Commit: git commit -m 'Add data with LFS: $DEST_SUBPATH'"
echo "  3. Push: git push"
echo ""
