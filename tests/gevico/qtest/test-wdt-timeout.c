/*
 * QTest: G233 watchdog timer
 *
 * Copyright (c) 2025 Chao Liu <chao.liu@yeah.net>
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * WDT register map (base 0x10010000):
 *   0x00  WDT_CTRL  — bit0: EN, bit1: INTEN
 *   0x04  WDT_LOAD  — reload value
 *   0x08  WDT_VAL   — current counter (read-only)
 *   0x0C  WDT_KEY   — write 0x5A5A5A5A=feed, 0x1ACCE551=lock
 *   0x10  WDT_SR    — bit0: TIMEOUT (write-1-to-clear)
 *
 * PLIC IRQ: 4
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define WDT_BASE    0x10010000ULL
#define WDT_CTRL    (WDT_BASE + 0x00)
#define WDT_LOAD    (WDT_BASE + 0x04)
#define WDT_VAL     (WDT_BASE + 0x08)
#define WDT_KEY     (WDT_BASE + 0x10)
#define WDT_SR      (WDT_BASE + 0x0C)

#define WDT_CTRL_EN     (1u << 0)
#define WDT_CTRL_INTEN  (1u << 1)

#define WDT_KEY_FEED    0x5A5A5A5A
#define WDT_KEY_LOCK    0x1ACCE551

#define WDT_SR_TIMEOUT  (1u << 0)

#define PLIC_BASE       0x0C000000ULL
#define PLIC_PENDING    (PLIC_BASE + 0x1000)
#define WDT_PLIC_IRQ    4

static inline bool plic_irq_pending(QTestState *qts, int irq)
{
    uint32_t word = qtest_readl(qts, PLIC_PENDING + (irq / 32) * 4);
    return (word >> (irq % 32)) & 1;
}

static void test_wdt_config(void)
{
    QTestState *qts = qtest_init("-machine g233 -m 2G");

    qtest_writel(qts, WDT_LOAD, 0x100);
    g_assert_cmpuint(qtest_readl(qts, WDT_LOAD), ==, 0x100);

    qtest_writel(qts, WDT_CTRL, WDT_CTRL_EN);
    g_assert_cmpuint(qtest_readl(qts, WDT_CTRL) & WDT_CTRL_EN, ==,
                     WDT_CTRL_EN);

    qtest_quit(qts);
}

static void test_wdt_countdown(void)
{
    QTestState *qts = qtest_init("-machine g233 -m 2G");

    qtest_writel(qts, WDT_LOAD, 0xFFFF);
    qtest_writel(qts, WDT_CTRL, WDT_CTRL_EN);

    uint32_t val1 = qtest_readl(qts, WDT_VAL);
    qtest_clock_step(qts, 10000000);  /* 10ms */
    uint32_t val2 = qtest_readl(qts, WDT_VAL);

    g_assert_cmpuint(val2, <, val1);

    qtest_quit(qts);
}

static void test_wdt_feed(void)
{
    QTestState *qts = qtest_init("-machine g233 -m 2G");

    qtest_writel(qts, WDT_LOAD, 0xFFFF);
    qtest_writel(qts, WDT_CTRL, WDT_CTRL_EN);

    /* Let it count down a bit */
    qtest_clock_step(qts, 10000000);
    uint32_t before_feed = qtest_readl(qts, WDT_VAL);

    /* Feed the watchdog */
    qtest_writel(qts, WDT_KEY, WDT_KEY_FEED);
    uint32_t after_feed = qtest_readl(qts, WDT_VAL);

    /* Counter should have been reloaded */
    g_assert_cmpuint(after_feed, >, before_feed);

    qtest_quit(qts);
}

static void test_wdt_timeout_flag(void)
{
    QTestState *qts = qtest_init("-machine g233 -m 2G");

    /* Small load value so it times out quickly */
    qtest_writel(qts, WDT_LOAD, 0x10);
    qtest_writel(qts, WDT_CTRL, WDT_CTRL_EN);

    /* Advance enough for timeout */
    qtest_clock_step(qts, 500000000);  /* 500ms */

    g_assert_cmpuint(qtest_readl(qts, WDT_SR) & WDT_SR_TIMEOUT, ==,
                     WDT_SR_TIMEOUT);

    qtest_quit(qts);
}

static void test_wdt_timeout_clear(void)
{
    QTestState *qts = qtest_init("-machine g233 -m 2G");

    qtest_writel(qts, WDT_LOAD, 0x10);
    qtest_writel(qts, WDT_CTRL, WDT_CTRL_EN);
    qtest_clock_step(qts, 500000000);

    g_assert_cmpuint(qtest_readl(qts, WDT_SR) & WDT_SR_TIMEOUT, !=, 0);

    /* Write 1 to clear */
    qtest_writel(qts, WDT_SR, WDT_SR_TIMEOUT);
    g_assert_cmpuint(qtest_readl(qts, WDT_SR) & WDT_SR_TIMEOUT, ==, 0);

    qtest_quit(qts);
}

static void test_wdt_lock(void)
{
    QTestState *qts = qtest_init("-machine g233 -m 2G");

    qtest_writel(qts, WDT_LOAD, 0xFFFF);
    qtest_writel(qts, WDT_CTRL, WDT_CTRL_EN);

    /* Lock the WDT */
    qtest_writel(qts, WDT_KEY, WDT_KEY_LOCK);

    /* Try to disable — should be ignored (locked) */
    qtest_writel(qts, WDT_CTRL, 0);
    g_assert_cmpuint(qtest_readl(qts, WDT_CTRL) & WDT_CTRL_EN, ==,
                     WDT_CTRL_EN);

    qtest_quit(qts);
}

static void test_wdt_interrupt(void)
{
    QTestState *qts = qtest_init("-machine g233 -m 2G");

    qtest_writel(qts, WDT_LOAD, 0x10);
    qtest_writel(qts, WDT_CTRL, WDT_CTRL_EN | WDT_CTRL_INTEN);

    qtest_clock_step(qts, 500000000);

    g_assert_cmpuint(qtest_readl(qts, WDT_SR) & WDT_SR_TIMEOUT, !=, 0);
    g_assert_true(plic_irq_pending(qts, WDT_PLIC_IRQ));

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("g233/wdt/config", test_wdt_config);
    qtest_add_func("g233/wdt/countdown", test_wdt_countdown);
    qtest_add_func("g233/wdt/feed", test_wdt_feed);
    qtest_add_func("g233/wdt/timeout_flag", test_wdt_timeout_flag);
    qtest_add_func("g233/wdt/timeout_clear", test_wdt_timeout_clear);
    qtest_add_func("g233/wdt/lock", test_wdt_lock);
    qtest_add_func("g233/wdt/interrupt", test_wdt_interrupt);

    return g_test_run();
}
