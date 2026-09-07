@echo off
set ZEPHYR_BASE=D:/ncs/v3.2.3/zephyr
set ZEPHYR_SDK_INSTALL_DIR=D:/ncs/toolchains/fd21892d0f/opt/zephyr-sdk
set ZEPHYR_TOOLCHAIN_VARIANT=zephyr
set PATH=D:/ncs/toolchains/fd21892d0f/opt/bin;D:/ncs/toolchains/fd21892d0f/opt/bin/Scripts;%PATH%
cd /d f:\Desktop\32project\kb
if "%1"=="-p" goto pristine
D:/ncs/toolchains/fd21892d0f/opt/bin/python.exe -m west build -b kb
goto end
:pristine
D:/ncs/toolchains/fd21892d0f/opt/bin/python.exe -m west build -p always -b kb
:end
