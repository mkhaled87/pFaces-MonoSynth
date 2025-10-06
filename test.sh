#!/bin/sh

# Testing example: ACC
cd examples/ACC
pfaces -CGH -k kernel.cpu@../../../kernel-pack -cfg ./acc.cfg -d 1 -p
cd ../..

