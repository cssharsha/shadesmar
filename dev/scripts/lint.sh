#!/bin/bash

bazel run //:buildifier
pre-commit run --all-files
