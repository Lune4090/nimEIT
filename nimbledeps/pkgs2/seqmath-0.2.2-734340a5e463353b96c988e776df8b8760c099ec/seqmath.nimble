# Package

version     = "0.2.2"
author      = "James Parkinson"
description = "math for sequences and nested sequences"
license     = "MIT"
srcDir      = "src"

requires "nim >= 1.0.0"

task test, "Run all tests":
  exec "nim c -r tests/tall.nim"
  exec "nim c -r tests/tHistogram.nim"
