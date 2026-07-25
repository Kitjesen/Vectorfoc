@echo off
REM VectorFOC Build Script for Windows
REM Usage: build.bat [app|boot|all|clean]

setlocal enabledelayedexpansion

set SCRIPT_DIR=%~dp0
set PROJECT_DIR=%SCRIPT_DIR%..
set TOOLCHAIN_FILE=%PROJECT_DIR%\cmake\gcc-arm-none-eabi.cmake

if "%1"=="" set TARGET=all
if not "%1"=="" set TARGET=%1

if "%TARGET%"=="app" goto build_app
if "%TARGET%"=="boot" goto build_boot
if "%TARGET%"=="all" goto build_all
if "%TARGET%"=="clean" goto clean_all
goto usage

:build_app
echo ==========================================
echo Building Application...
echo ==========================================

if not exist "%PROJECT_DIR%\build" mkdir "%PROJECT_DIR%\build"
cd /d "%PROJECT_DIR%\build"

cmake -S "%PROJECT_DIR%" -B "%PROJECT_DIR%\build" -G "MinGW Makefiles" -DCMAKE_TOOLCHAIN_FILE="%TOOLCHAIN_FILE%" -DCMAKE_BUILD_TYPE=Release -DBOOTLOADER_BUILD=OFF
if errorlevel 1 goto error

cmake --build "%PROJECT_DIR%\build" --parallel 4
if errorlevel 1 goto error

echo.
echo Application built: build\VectorFoc.bin
goto end

:build_boot
echo ==========================================
echo Building Bootloader...
echo ==========================================

cmake -S "%PROJECT_DIR%" -B "%PROJECT_DIR%\build_boot" -G "MinGW Makefiles" -DCMAKE_TOOLCHAIN_FILE="%TOOLCHAIN_FILE%" -DCMAKE_BUILD_TYPE=Release -DBOOTLOADER_BUILD=ON
if errorlevel 1 goto error

cmake --build "%PROJECT_DIR%\build_boot" --parallel 4
if errorlevel 1 goto error

echo.
echo Bootloader built: build_boot\VectorFoc_Bootloader.bin
goto end

:build_all
call :build_boot
call :build_app
goto end

:clean_all
echo Cleaning build directories...
if exist "%PROJECT_DIR%\build" rmdir /s /q "%PROJECT_DIR%\build"
if exist "%PROJECT_DIR%\build_boot" rmdir /s /q "%PROJECT_DIR%\build_boot"
echo Done.
goto end

:usage
echo Usage: %0 [app^|boot^|all^|clean]
echo   app   - Build application only
echo   boot  - Build bootloader only
echo   all   - Build both (default)
echo   clean - Clean build directories
goto end

:error
echo.
echo Build failed!
exit /b 1

:end
echo.
echo Build complete!
