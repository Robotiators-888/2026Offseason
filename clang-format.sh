#!/usr/bin/sh
find src/main/java/frc/robot -exec clang-format -i -style=file --assume-filename java {} +