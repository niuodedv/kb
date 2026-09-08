/*
 * 软件时钟模块实现（LCD 第二行时间 + 上位机同步预留）
 *
 * 设计：
 *   epoch_now = base_epoch + uptime(秒)
 *   其中 base_epoch 在 kb_time_init / kb_time_set 系列（kb_time_set_epoch 等）时写入，
 *   uptime 秒由 Zephyr k_uptime_get() 提供（32.768k 晶振走时）。
 *   用纯整数 "days_from_civil / civil_from_days"（Howard Hinnant 算法）
 *   做 epoch <-> 年月日换算，避免依赖标准库的时区/线程安全问题。
 *
 * 详见 lcd 显示.txt【6】
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include "kb_time.h"

#include <errno.h>
#include <stdbool.h>

LOG_MODULE_REGISTER(kb_time, LOG_LEVEL_INF);

/* 上电默认时间（掉电不保持，等上位机/串口同步覆盖） */
#define KB_TIME_DEFAULT_YEAR 2026u
#define KB_TIME_DEFAULT_MON  1u
#define KB_TIME_DEFAULT_DAY  1u
#define KB_TIME_DEFAULT_HOUR 0u
#define KB_TIME_DEFAULT_MIN  0u
#define KB_TIME_DEFAULT_SEC  0u

#define SECS_PER_MIN 60u
#define SECS_PER_HOUR 3600u
#define SECS_PER_DAY (24u * SECS_PER_HOUR)

/* 同步基点：默认时间的 epoch 秒 + 系统启动以来流逝的秒数 = 当前 epoch */
static int64_t base_epoch;

/* ---------------- 公历换算（Hinnant 算法，输入/输出均为公历） ---------------- */

/* 从 y/m/d 得到「距 1970-01-01 的天数」 */
static int64_t days_from_civil(int y, unsigned int m, unsigned int d)
{
	y -= (int)(m <= 2);
	int64_t era = (y >= 0 ? y : y - 399) / 400;
	unsigned int yoe = (unsigned int)(y - era * 400); /* [0, 399] */
	unsigned int doy = (153u * (m + (m > 2 ? -3u : 9u)) + 2u) / 5u + d - 1u;
	unsigned int doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;

	return era * 146097 + (int64_t)doe - 719468;
}

/* 从「距 1970-01-01 的天数」反算 y/m/d */
static void civil_from_days(int64_t z, int *y, int *m, int *d)
{
	z += 719468;
	int64_t era = (z >= 0 ? z : z - 146096) / 146097;
	unsigned int doe = (unsigned int)(z - era * 146097);          /* [0, 146096] */
	unsigned int yoe = (doe - doe / 1460u + doe / 36524u - doe / 146096u) / 365u;
	int64_t y_out = (int64_t)yoe + era * 400;
	unsigned int doy = doe - (365u * yoe + yoe / 4u - yoe / 100u);
	unsigned int mp = (5u * doy + 2u) / 153u;
	unsigned int d_out = doy - (153u * mp + 2u) / 5u + 1u;
	unsigned int m_out = mp < 10u ? mp + 3u : mp - 9u;

	y_out += (m_out <= 2u);
	*y = (int)y_out;
	*m = (int)m_out;
	*d = (int)d_out;
}

static int64_t to_epoch(const struct kb_time *t)
{
	return days_from_civil(t->year, t->month, t->day) * SECS_PER_DAY +
	       (int64_t)t->hour * SECS_PER_HOUR +
	       (int64_t)t->minute * SECS_PER_MIN +
	       t->second;
}

static int time_valid(const struct kb_time *t)
{
	if (!t) {
		return 0;
	}
	if (t->year < 2000 || t->year > 9999) {
		return 0;
	}
	if (t->month < 1 || t->month > 12) {
		return 0;
	}
	if (t->day < 1 || t->day > 31) {
		return 0;
	}
	if (t->hour > 23 || t->minute > 59 || t->second > 59) {
		return 0;
	}
	return 1;
}

/* ---------------- 对外接口 ---------------- */

int kb_time_init(void)
{
	struct kb_time t = {
		.year = KB_TIME_DEFAULT_YEAR,
		.month = KB_TIME_DEFAULT_MON,
		.day = KB_TIME_DEFAULT_DAY,
		.hour = KB_TIME_DEFAULT_HOUR,
		.minute = KB_TIME_DEFAULT_MIN,
		.second = KB_TIME_DEFAULT_SEC,
	};

	base_epoch = to_epoch(&t);
	LOG_INF("软件时钟已初始化（基点 %u-%02u-%02u %02u:%02u:%02u）",
		(unsigned)t.year, (unsigned)t.month, (unsigned)t.day,
		(unsigned)t.hour, (unsigned)t.minute, (unsigned)t.second);
	return 0;
}

int kb_time_set(const struct kb_time *t)
{
	if (!time_valid(t)) {
		return -EINVAL;
	}

	int64_t now = base_epoch + k_uptime_get() / 1000;

	base_epoch = to_epoch(t) - now;
	return 0;
}

int kb_time_set_epoch(uint32_t sec)
{
	base_epoch = (int64_t)sec - k_uptime_get() / 1000;
	return 0;
}

int kb_time_get(struct kb_time *t)
{
	int64_t now;
	int64_t days;
	int64_t secs_of_day;
	int y, m, d;

	if (!t) {
		return -EINVAL;
	}

	now = base_epoch + k_uptime_get() / 1000;
	days = now / SECS_PER_DAY;
	secs_of_day = now % SECS_PER_DAY;
	if (secs_of_day < 0) {
		secs_of_day += SECS_PER_DAY;
		days -= 1;
	}

	civil_from_days(days, &y, &m, &d);
	t->year = (uint16_t)y;
	t->month = (uint8_t)m;
	t->day = (uint8_t)d;
	t->hour = (uint8_t)(secs_of_day / SECS_PER_HOUR);
	t->minute = (uint8_t)((secs_of_day % SECS_PER_HOUR) / SECS_PER_MIN);
	t->second = (uint8_t)(secs_of_day % SECS_PER_MIN);

	return 0;
}

/* ---------------- shell：time get / time set ---------------- */

static int cmd_time_get(const struct shell *sh, size_t argc, char **argv)
{
	struct kb_time t;

	(void)argc;
	(void)argv;

	if (kb_time_get(&t) != 0) {
		shell_error(sh, "读取时间失败");
		return -EIO;
	}
	shell_print(sh, "%04u-%02u-%02u %02u:%02u:%02u",
		    (unsigned)t.year, (unsigned)t.month, (unsigned)t.day,
		    (unsigned)t.hour, (unsigned)t.minute, (unsigned)t.second);
	return 0;
}

static int cmd_time_set(const struct shell *sh, size_t argc, char **argv)
{
	struct kb_time t;
	int err = 0;

	if (argc != 7) {
		shell_error(sh, "用法: time set <yyyy> <mm> <dd> <HH> <MM> <SS>");
		return -EINVAL;
	}

	t.year = (uint16_t)shell_strtoul(argv[1], 10, &err);
	t.month = (uint8_t)shell_strtoul(argv[2], 10, &err);
	t.day = (uint8_t)shell_strtoul(argv[3], 10, &err);
	t.hour = (uint8_t)shell_strtoul(argv[4], 10, &err);
	t.minute = (uint8_t)shell_strtoul(argv[5], 10, &err);
	t.second = (uint8_t)shell_strtoul(argv[6], 10, &err);

	if (err != 0) {
		shell_error(sh, "参数不是数字");
		return -EINVAL;
	}

	if (!time_valid(&t)) {
		shell_error(sh, "时间字段超出范围");
		return -EINVAL;
	}

	kb_time_set(&t);
	shell_print(sh, "时间已同步: %04u-%02u-%02u %02u:%02u:%02u",
		    (unsigned)t.year, (unsigned)t.month, (unsigned)t.day,
		    (unsigned)t.hour, (unsigned)t.minute, (unsigned)t.second);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_time,
	SHELL_CMD(get, NULL, "读取当前时间 (yyyy-mm-dd HH:MM:SS)", cmd_time_get),
	SHELL_CMD(set, NULL, "设置时间: time set <yyyy> <mm> <dd> <HH> <MM> <SS>",
		  cmd_time_set),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(time, &sub_time, "软件时钟（LCD 时间源）",
		   NULL);
