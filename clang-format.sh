#!/usr/bin/sh
find . -exec clang-format -i -style=file --assume-filename java {} +