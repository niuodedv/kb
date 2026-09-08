/*
 * LCD 5x7 ASCII 子集字体（按需收录，放 flash）
 *
 * 数据结构：每个字形 = 7 个字节，每字节 = 1 行像素（行从上到下），
 *           每字节的低 5 位 = 第 0..4 列（bit0=最左列）。
 * 收录字符：数字 0-9、空格、: - . % 以及模式标签用大写 USBEGL。
 */

#pragma once

#include <stdint.h>

#define KB_FONT_W 5u
#define KB_FONT_H 7u

/**
 * @brief 取字形数据（7 字节），未收录返回 NULL
 */
const uint8_t *kb_lcd_font_glyph(char ch);
