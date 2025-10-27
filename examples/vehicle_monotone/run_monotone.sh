#!/bin/bash

# Run pFaces with monotone synthesis
# This script runs the vehicle example with verbose output to show the synthesis progress

echo "========================================================="
echo "Running Vehicle Example with Monotone Synthesis"
echo "========================================================="
echo ""
echo "This will:"
echo "  1. Run parallel abstraction on GPU/CPU"
echo "  2. Run monotone synthesis on host (our new function)"
echo "  3. Save the controller"
echo ""
echo "Using configuration: vehicle_monotone.cfg"
echo ""

# Get the pfaces binary path
PFACES_BIN="../../../../bin/pfaces"

# Check if pfaces exists
if [ ! -f "$PFACES_BIN" ]; then
    echo "Error: pfaces not found at $PFACES_BIN"
    echo "Please run from the examples directory or adjust the path"
    exit 1
fi

# Run with CPU device (change -d 1 to use a different device)
# -CG: Use CPU and GPU
# -k: Specify kernel name
# -cfg: Configuration file
# -d: Device ID (1 = first CPU, or use 'pfaces -CGH -l' to list devices)
# -v: Verbosity level (2 = detailed output)
# -p: Profile timing

echo "Executing command:"
echo "$PFACES_BIN -CG -k mono_synth@../../kernel-pack -cfg ./vehicle_monotone.cfg -d 1 -v 2"
echo ""
echo "Starting execution..."
echo ""

$PFACES_BIN -CG -k mono_synth.cpu@../../kernel-pack -cfg ./vehicle_monotone.cfg -d 1 -v 2

echo ""
echo "========================================================="
echo "Execution complete!"
echo "========================================================="
echo ""
echo "Output files:"
echo "  - vehicle_monotone.raw: Controller data"
echo ""
echo "Check the output above for:"
echo "  - 'Starting monotone synthesis algorithm...'"
echo "  - 'Monotone synthesis complete (placeholder implementation)'"
echo "  - This confirms our synthesis function is being called!"

