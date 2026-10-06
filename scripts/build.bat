@echo off
rem VectorFOC offline build helper: build.bat [app|boot|all|clean]
setlocal
set "PROJECT_DIR=%~dp0.."
set "TOOLCHAIN_FILE=%PROJECT_DIR%\cmake\gcc-arm-none-eabi.cmake"
set "TARGET=%~1"
if "%TARGET%"=="" set "TARGET=app"

if /I "%TARGET%"=="app" goto run_app
if /I "%TARGET%"=="boot" goto run_boot
if /I "%TARGET%"=="all" goto run_all
if /I "%TARGET%"=="clean" goto run_clean
echo Usage: %~nx0 [app^|boot^|all^|clean]
exit /b 1

:run_app
call :build_app
exit /b %errorlevel%

:run_boot
call :build_boot
exit /b %errorlevel%

:run_all
call :build_boot
if errorlevel 1 exit /b 1
call :build_app
exit /b %errorlevel%

:run_clean
for %%T in (arm host boot keil) do (
    call :clean_target %%T
    if errorlevel 1 exit /b 1
)
exit /b 0

:build_app
cmake -S "%PROJECT_DIR%" -B "%PROJECT_DIR%\build\arm" -G Ninja --toolchain "%TOOLCHAIN_FILE%" -DCMAKE_BUILD_TYPE=Release
if errorlevel 1 exit /b 1
cmake --build "%PROJECT_DIR%\build\arm" --parallel 4
if errorlevel 1 exit /b 1
echo Application built: build\arm\VectorFoc.bin
exit /b 0

:build_boot
cmake -S "%PROJECT_DIR%\cmake\bootloader" -B "%PROJECT_DIR%\build\boot" -G Ninja --toolchain "%TOOLCHAIN_FILE%" -DCMAKE_BUILD_TYPE=Release
if errorlevel 1 exit /b 1
cmake --build "%PROJECT_DIR%\build\boot" --parallel 4
if errorlevel 1 exit /b 1
echo Bootloader built: build\boot\VectorFoc_Bootloader.bin
exit /b 0

:clean_target
if not exist "%PROJECT_DIR%\build\%~1\CMakeCache.txt" exit /b 0
cmake --build "%PROJECT_DIR%\build\%~1" --target clean
exit /b %errorlevel%
