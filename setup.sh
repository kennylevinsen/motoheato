#!/bin/bash
set -e

# Use local directory for tools
export IDF_TOOLS_PATH=$(pwd)/.espressif
mkdir -p $IDF_TOOLS_PATH

echo "Installing ESP-IDF tools locally to $IDF_TOOLS_PATH..."
./esp-idf/install.sh esp32c6

echo "Setup complete. To build, run:"
echo "export IDF_TOOLS_PATH=$(pwd)/.espressif"
echo ". ./esp-idf/export.sh"
echo "idf.py build"
