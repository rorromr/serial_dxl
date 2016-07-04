#!/bin/bash

# Source tools
source $1/scripts/build_tool.sh
# Build GoKart examples
build_examples $1 arduino:avr:uno
