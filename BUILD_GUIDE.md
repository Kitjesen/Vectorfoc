cd D:\robot\FOC\FalconFoc-main\2.Firmware\FalconFoc

# 1) 彻底清理旧缓存（非常关键）

if (Test-Path .\build) { Remove-Item -Recurse -Force .\build }
New-Item -ItemType Directory -Path .\build | Out-Null

# 2) 当前会话临时把 mingw64 放到 PATH 最前（避免被 clang64 的 mingw32-make 抢走）

$env:Path = "D:\msys64\mingw64\bin;" + $env:Path

# 3) 重新配置：用 --toolchain 强制进入裸机交叉编译模式，并钉死 make 程序

cmake -S . -B build -G "MinGW Makefiles" `  --toolchain cmake/gcc-arm-none-eabi.cmake`
  -DCMAKE_MAKE_PROGRAM=D:\msys64\mingw64\bin\mingw32-make.exe `  -DCMAKE_BUILD_TYPE=Debug`
  -DCMAKE_TRY_COMPILE_TARGET_TYPE=STATIC_LIBRARY `  -DCMAKE_SYSTEM_NAME=Generic`
  -DCMAKE_C_COMPILER="D:/Program Files/armcc/xpack-arm-none-eabi-gcc/bin/arm-none-eabi-gcc.exe" `  -DCMAKE_CXX_COMPILER="D:/Program Files/armcc/xpack-arm-none-eabi-gcc/bin/arm-none-eabi-g++.exe"`
  -DCMAKE_ASM_COMPILER="D:/Program Files/armcc/xpack-arm-none-eabi-gcc/bin/arm-none-eabi-gcc.exe"

# 4) 编译

cmake --build build -j 8

cmake --build build --clean-first
