#!/bin/bash
# Setup git hooks for linux kernel

ln -s -f $(pwd)/hooks/post-commit .git/hooks/post-commit

