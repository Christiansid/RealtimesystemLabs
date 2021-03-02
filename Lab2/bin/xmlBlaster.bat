@echo off
pushd %~dp0
java -jar ..\lib\xmlBlaster.jar -hostname JGrafchart -port 3412 %*
popd

