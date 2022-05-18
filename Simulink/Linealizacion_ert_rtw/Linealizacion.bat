
set MATLAB=C:\Program Files (x86)\Polyspace\R2021a

cd .

chcp 1252

if "%1"=="" ("C:\PROGRA~2\POLYSP~1\R2021a\bin\win64\gmake"  -f Linealizacion.mk all) else ("C:\PROGRA~2\POLYSP~1\R2021a\bin\win64\gmake"  -f Linealizacion.mk %1)
@if errorlevel 1 goto error_exit

exit /B 0

:error_exit
echo The make command returned an error of %errorlevel%
exit /B 1