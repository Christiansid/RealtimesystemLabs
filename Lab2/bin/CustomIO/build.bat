@echo off

REM Your Custom IO package name, e.g. myCustomIO (means grafchart.sfc.io.myCustomIO)
set customIOPackage=

REM ----------------------------------------------------------------------------

set src=../../source
set out=../../Code

javac -classpath %src% %src%/grafchart/sfc/io/%customIOPackage%/*.java -d %out%
