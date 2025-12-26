#!/bin/bash
set -e

export IDF_TOOLS_PATH=$(pwd)/.espressif
. ./esp-idf/export.sh

idf.py build "$@"
