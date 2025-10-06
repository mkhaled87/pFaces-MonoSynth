@ECHO OFF

rem Testing example: ACC
cd examples\acc
pfaces -CGH -k kernel.cpu@..\..\kernel-pack -cfg .\acc.cfg -d 1 -p
cd ..\..
