#!/bin/bash
set -e  # Exit on any error

# Check if git repository is clean
# if [ -n "$(git status --porcelain)" ]; then
#     echo "Working directory is not clean. Please commit all changes before running this script."
#     exit 1
# fi

# Prompt for confirmation
read -p "This script will modify source files. Are you sure you want to continue? (y/N) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    exit 1
fi

# Create backup of source files
echo "Creating backup of source files..."
timestamp=$(date +%Y%m%d_%H%M%S)
backup_dir="./backup_${timestamp}"
mkdir -p "$backup_dir"
find . \( -name '*.cpp' -o -name '*.hpp' -o -name '*.cc' -o -name '*.h' \) \
    -not -path "./bazel-*/*" \
    -not -path "./external/*" \
    -not -path "./third_party/*" \
    -exec cp --parents {} "$backup_dir" \;

echo "Running pre-commit hooks..."
pre-commit run --all-files

echo "Checking and fixing C++ formatting..."
find . \( -name '*.cpp' -o -name '*.hpp' -o -name '*.cc' -o -name '*.h' \) \
    -not -path "./bazel-*/*" \
    -not -path "./external/*" \
    -not -path "./third_party/*" \
    | xargs clang-format --style=file -i

echo "Running clang-tidy with automatic fixes..."
# Generate compile_commands.json
bazel run @hedron_compile_commands//:refresh_all

# Run clang-tidy with automatic fixes
find . \( -name '*.cpp' -o -name '*.hpp' -o -name '*.cc' -o -name '*.h' \) \
    -not -path "./bazel-*/*" \
    -not -path "./external/*" \
    -not -path "./third_party/*" \
    | xargs clang-tidy -p compile_commands.json --fix-errors

echo "Building project..."
bazel build //...

echo "Running tests..."
bazel test //... --test_output=errors

# Cleanup section
echo "Cleaning up..."
# Remove backup if everything succeeded
read -p "All checks passed! Do you want to remove the backup files? (y/N) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    rm -rf "$backup_dir"
    echo "Backup files removed."
else
    echo "Backup files kept at: $backup_dir"
fi

# Clean bazel artifacts
read -p "Do you want to clean Bazel artifacts? (y/N) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    bazel clean
    echo "Bazel artifacts cleaned."
fi

# Show git status after all modifications
echo "Changes made to your files:"
git status

echo "All checks passed and cleanup completed!"
