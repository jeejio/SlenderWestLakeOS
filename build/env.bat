@echo off
if not "%_env_init%"=="" goto :__to_init
set _env_init=1
set JEEJIO_PATH=%~dp0..
set target_dir=%JEEJIO_PATH%\out\target\esp32c3
set PATH=%JEEJIO_PATH%\tools\python;%JEEJIO_PATH%\tools\ccache;%JEEJIO_PATH%\tools\cmake\bin;%JEEJIO_PATH%\tools\ninja;%JEEJIO_PATH%\tools\python\Scripts;%JEEJIO_PATH%\tools\riscv32-esp-elf\bin;%PATH%
set JEEJIOBAUD=460800

doskey config=ninja -C %target_dir% menuconfig
doskey build=ninja -C %target_dir%
doskey flash=ninja -C %target_dir% flash
doskey monitor=ninja -C %target_dir% monitor
doskey clean=RD /S /Q "%JEEJIO_PATH%\out"

:__to_init

if not exist %target_dir% mkdir %target_dir%
if not exist %target_dir%\build.ninja (
cd %target_dir%
cmake -G Ninja -DESP_PLATFORM=1 -DJEEJIO_TARGET=esp32c3 -DCCACHE_ENABLE=1 %JEEJIO_PATH%
cd %JEEJIO_PATH%
)