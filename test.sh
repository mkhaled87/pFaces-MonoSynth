#!/bin/sh

# Testing example: ACC
cd examples/turn_ego_first
pfaces -GH -k mono_synth.cpu@../../kernel-pack -cfg ./turn_ego_first.cfg -d 1 -p -v1
cd ../..

