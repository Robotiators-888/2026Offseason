#!/usr/bin/sh
find src/main/java/frc/robot -type f -exec clang-format -i -style=file --assume-filename java {} +