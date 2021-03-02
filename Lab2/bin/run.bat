@echo off
setlocal
set LIB=../lib
set CODE=../code
set CoP="%CODE%;%LIB%/JGrafchart.jar;%LIB%/JGo.jar;%LIB%/regler.jar;%LIB%/xmlBlaster.jar;%LIB%/jutils.jar;%LIB%/jhall.jar;%LIB%/CCOM.jar;%LIB%/dpws/dpws4j.jar;%LIB%/dpws/soda-JettyServletContainer.jar;%LIB%/dpws/soda-jdom.jar;%LIB%/dpws/wsdl4j-1.6.1.jar;%LIB%/beaver-rt.jar"

:: Symlink support would be messy, see http://stackoverflow.com/questions/13298414/native-alternative-for-readlink-on-windows
dir "%~f0" | find "<SYMLINK>" > NUL
if %ERRORLEVEL% == 0 (
	echo ERROR: JGrafchart start script does not support symlinks on Windows. Use a shortcut instead.
	pause
	exit /b 1
)

pushd %~dp0
java -Xincgc -classpath %CoP% grafchart.sfc.Editor %*
if %ERRORLEVEL% == 9009 (
	echo ERROR: Java is not installed properly.
	pause
) else if not %ERRORLEVEL% == 0 (
	echo ERROR: JGrafchart exited with error code %ERRORLEVEL%.
	pause
)
popd
