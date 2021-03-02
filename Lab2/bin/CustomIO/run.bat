@echo off

REM Your Custom IO package name, e.g. myCustomIO (means grafchart.sfc.io.myCustomIO)
set customIOPackage=
REM Your Custom IO class name, i.e. the class implementing the LocalIO interface
set customIOClass=

REM ----------------------------------------------------------------------------

%~dp0..\run.bat -localIO grafchart.sfc.io.%customIOPackage%.%customIOClass% %*
