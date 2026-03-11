#!/bin/sh

# Testing example: ACC
cd examples/acc
pfaces -GH -k mono_synth.cpu@../../kernel-pack -cfg ./acc.cfg -d 1 -p -v1
cd ../..

