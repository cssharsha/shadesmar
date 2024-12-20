def _clang_format_impl(ctx):
    script = ctx.actions.declare_file(ctx.label.name + ".sh")

    content = """#!/bin/bash
set -euo pipefail

# Format files and store output in a temporary file
find . \\( -name '*.cpp' -o -name '*.hpp' -o -name '*.cc' -o -name '*.h' \\) \\
    -not -path "./bazel-*/*" \\
    -not -path "./external/*" \\
    -not -path "./third_party/*" \\
    | while read file; do
        # Create a temp file
        temp_file=$(mktemp)
        # Format to temp file
        clang-format --style=file "$file" > "$temp_file"
        # Compare with original
        if ! diff "$file" "$temp_file" > /dev/null; then
            echo "File $file needs formatting"
            rm "$temp_file"
            exit 1
        fi
        rm "$temp_file"
    done
"""

    ctx.actions.write(
        output = script,
        content = content,
        is_executable = True,
    )

    return [DefaultInfo(
        executable = script,
        runfiles = ctx.runfiles(files = []),
    )]

clang_format_test = rule(
    implementation = _clang_format_impl,
    test = True,
)

def _clang_format_fix_impl(ctx):
    script = ctx.actions.declare_file(ctx.label.name + ".sh")

    content = """#!/bin/bash
set -euo pipefail

# Find all C++ files and format them
find . \\( -name '*.cpp' -o -name '*.hpp' -o -name '*.cc' -o -name '*.h' \\) \\
    -not -path "./bazel-*/*" \\
    -not -path "./external/*" \\
    -not -path "./third_party/*" \\
    | while read file; do
        clang-format -i --style=file "$file"
    done
"""

    ctx.actions.write(
        output = script,
        content = content,
        is_executable = True,
    )

    return [DefaultInfo(
        executable = script,
        runfiles = ctx.runfiles(files = []),
    )]

clang_format_fix = rule(
    implementation = _clang_format_fix_impl,
    executable = True,
)
