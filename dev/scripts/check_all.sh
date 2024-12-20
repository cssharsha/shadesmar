#!/bin/bash
set -e  # Exit on any error

echo "Running pre-commit hooks..."
pre-commit run --all-files

echo "Checking C++ formatting..."
find . \( -name '*.cpp' -o -name '*.hpp' -o -name '*.cc' -o -name '*.h' \) \
    -not -path "./bazel-*/*" \
    -not -path "./external/*" \
    -not -path "./third_party/*" \
    | xargs clang-format --style=file --dry-run -Werror

echo "Running clang-tidy..."
bear -- bazel build //...
find . \( -name '*.cpp' -o -name '*.hpp' -o -name '*.cc' -o -name '*.h' \) \
    -not -path "./bazel-*/*" \
    -not -path "./external/*" \
    -not -path "./third_party/*" \
    | xargs clang-tidy -p compile_commands.json

echo "Building project..."
bazel build //...

echo "Running tests..."
bazel test //... --test_output=errors

echo "All checks passed!"
