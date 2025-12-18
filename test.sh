#!/bin/sh

# Testing example: ACC
cd examples/ACC
pfaces -CGH -k mono_synth.cpu@../../kernel-pack -cfg ./acc.cfg -d 1 -p -v4
cd ../..

