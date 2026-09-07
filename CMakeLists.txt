cmake_minimum_required(VERSION 3.20.0)
# 自定义板 kb 位于本工程 boards/kb/kb/，需将工程根目录加入 BOARD_ROOT，
# 让 Zephyr 在 find_package 前能找到该板（避免命令行传绝对路径时盘符冒号被拆分）。
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
# 自定义设备树 binding（dts/bindings/injoinic,ip5306.yaml）需要 DTS_ROOT
# 才能被 edtlib 找到。
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(kb)

target_include_directories(app PRIVATE src)

target_sources(app PRIVATE
	src/main.c
	src/keys/keys.c
	src/led/led.c
	src/mode/mode.c
	src/knob/knob.c
	src/hid/hid_keymap.c
	src/ble/ble_hid.c
	src/usb/usb_hid.c
	src/power/ip5306.c
	src/power/battery.c
	src/power/bat_adc.c
)
