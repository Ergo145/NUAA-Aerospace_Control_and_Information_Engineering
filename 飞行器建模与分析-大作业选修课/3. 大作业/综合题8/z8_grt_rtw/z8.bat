
set MATLAB=E:\Matlab

cd .

if "%1"=="" ("E:\Matlab\bin\win64\gmake"  -f z8.mk all) else ("E:\Matlab\bin\win64\gmake"  -f z8.mk %1)
@if errorlevel 1 goto error_exit

exit /B 0

:error_exit
echo The make command returned an error of %errorlevel%
exit /B 1