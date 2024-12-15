#!/bin/bash

# Install dependencies
sudo apt-get update && sudo apt-get install -y \
    clang-format \
    clang-tidy \
    cmake \
    ninja-build \
    pre-commit
