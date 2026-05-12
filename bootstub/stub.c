/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <stdint.h>

#ifndef APP_BASE
#define APP_BASE 0x00008000u
#endif

#define SRAM_BASE 0x20000000u
#define SRAM_SIZE 0x00020000u
#define STACK_TOP (SRAM_BASE + SRAM_SIZE)

#define SYST_CSR (*(volatile uint32_t *)0xE000E010u)
#define SCB_VTOR (*(volatile uint32_t *)0xE000ED08u)
#define NVIC_ICER ((volatile uint32_t *)0xE000E180u)
#define NVIC_ICPR ((volatile uint32_t *)0xE000E280u)

#ifndef BOOTSTUB_UART_BANNER
#define BOOTSTUB_UART_BANNER 1
#endif

#define UARTE0_BASE 0x40002000u
#define UARTE0_TASKS_STARTTX (*(volatile uint32_t *)(UARTE0_BASE + 0x008u))
#define UARTE0_TASKS_STOPTX (*(volatile uint32_t *)(UARTE0_BASE + 0x00Cu))
#define UARTE0_EVENTS_ENDTX (*(volatile uint32_t *)(UARTE0_BASE + 0x120u))
#define UARTE0_EVENTS_TXSTOPPED (*(volatile uint32_t *)(UARTE0_BASE + 0x158u))
#define UARTE0_ENABLE (*(volatile uint32_t *)(UARTE0_BASE + 0x500u))
#define UARTE0_PSEL_RTS (*(volatile uint32_t *)(UARTE0_BASE + 0x508u))
#define UARTE0_PSEL_TXD (*(volatile uint32_t *)(UARTE0_BASE + 0x50Cu))
#define UARTE0_PSEL_CTS (*(volatile uint32_t *)(UARTE0_BASE + 0x510u))
#define UARTE0_PSEL_RXD (*(volatile uint32_t *)(UARTE0_BASE + 0x514u))
#define UARTE0_BAUDRATE (*(volatile uint32_t *)(UARTE0_BASE + 0x524u))
#define UARTE0_TXD_PTR (*(volatile uint32_t *)(UARTE0_BASE + 0x544u))
#define UARTE0_TXD_MAXCNT (*(volatile uint32_t *)(UARTE0_BASE + 0x548u))
#define UARTE0_CONFIG (*(volatile uint32_t *)(UARTE0_BASE + 0x56Cu))

#define GPIO0_BASE 0x50000000u
#define GPIO1_BASE 0x50000300u
#define GPIO_PIN_CNF(base, pin) (*(volatile uint32_t *)((base) + 0x700u + ((pin) * 4u)))

#define GPIO_CNF_DIR_INPUT 0u
#define GPIO_CNF_DIR_OUTPUT 1u
#define GPIO_CNF_INPUT_CONNECT (0u << 1)
#define GPIO_CNF_INPUT_DISCONNECT (1u << 1)
#define GPIO_CNF_PULL_DISABLED (0u << 2)
#define GPIO_CNF_PULL_UP (3u << 2)
#define GPIO_CNF_DRIVE_S0S1 (0u << 8)
#define GPIO_CNF_SENSE_DISABLED (0u << 16)

#define UARTE_ENABLE_ENABLED 8u
#define UARTE_BAUDRATE_115200 0x01D7E000u
#define UARTE_PSEL_DISCONNECTED 0xFFFFFFFFu
#define UARTE_TXD_PIN 6u
#define UARTE_RXD_PIN (32u + 8u)
#define MICROBIT_BUTTON_A_PIN 14u
#define MICROBIT_BUTTON_B_PIN 23u

typedef void (*entry_fn)(void);

void reset_handler(void);
void hang(void);

__attribute__((section(".vectors"), used)) const uintptr_t vector_table[] = {
	STACK_TOP,
	(uintptr_t)reset_handler,
	(uintptr_t)hang, /* NMI */
	(uintptr_t)hang, /* HardFault */
	(uintptr_t)hang, /* MemManage */
	(uintptr_t)hang, /* BusFault */
	(uintptr_t)hang, /* UsageFault */
	0,
	0,
	0,
	0,
	(uintptr_t)hang, /* SVCall */
	(uintptr_t)hang, /* DebugMonitor */
	0,
	(uintptr_t)hang, /* PendSV */
	(uintptr_t)hang, /* SysTick */
};

void hang(void)
{
	for(;;)
	{
		__asm volatile("wfi");
	}
}

#if BOOTSTUB_UART_BANNER
static void uart0_putc(char ch)
{
	volatile uint8_t byte = (uint8_t)ch;

	UARTE0_EVENTS_ENDTX = 0;
	UARTE0_TXD_PTR = (uint32_t)&byte;
	UARTE0_TXD_MAXCNT = 1;
	UARTE0_TASKS_STARTTX = 1;

	while(UARTE0_EVENTS_ENDTX == 0)
	{
	}
}

static void uart0_puts(const char *s)
{
	while(*s != '\0')
	{
		uart0_putc(*s++);
	}
}

static void uart0_banner(void)
{
	GPIO_PIN_CNF(GPIO0_BASE, 6) = GPIO_CNF_DIR_OUTPUT |
	                              GPIO_CNF_INPUT_DISCONNECT |
	                              GPIO_CNF_PULL_DISABLED |
	                              GPIO_CNF_DRIVE_S0S1 |
	                              GPIO_CNF_SENSE_DISABLED;
	GPIO_PIN_CNF(GPIO1_BASE, 8) = GPIO_CNF_DIR_INPUT |
	                              GPIO_CNF_INPUT_CONNECT |
	                              GPIO_CNF_PULL_DISABLED |
	                              GPIO_CNF_DRIVE_S0S1 |
	                              GPIO_CNF_SENSE_DISABLED;

	UARTE0_ENABLE = 0;
	UARTE0_PSEL_RTS = UARTE_PSEL_DISCONNECTED;
	UARTE0_PSEL_TXD = UARTE_TXD_PIN;
	UARTE0_PSEL_CTS = UARTE_PSEL_DISCONNECTED;
	UARTE0_PSEL_RXD = UARTE_RXD_PIN;
	UARTE0_CONFIG = 0;
	UARTE0_BAUDRATE = UARTE_BAUDRATE_115200;
	UARTE0_ENABLE = UARTE_ENABLE_ENABLED;

	uart0_puts("\r\nibex microbit bootstub\r\n");

	UARTE0_EVENTS_TXSTOPPED = 0;
	UARTE0_TASKS_STOPTX = 1;
	while(UARTE0_EVENTS_TXSTOPPED == 0)
		;
	UARTE0_ENABLE = 0;
}
#endif

static void microbit_buttons_init(void)
{
	const uint32_t input_pullup = GPIO_CNF_DIR_INPUT |
	                              GPIO_CNF_INPUT_CONNECT |
	                              GPIO_CNF_PULL_UP |
	                              GPIO_CNF_DRIVE_S0S1 |
	                              GPIO_CNF_SENSE_DISABLED;

	GPIO_PIN_CNF(GPIO0_BASE, MICROBIT_BUTTON_A_PIN) = input_pullup;
	GPIO_PIN_CNF(GPIO0_BASE, MICROBIT_BUTTON_B_PIN) = input_pullup;
}

__attribute__((noreturn)) void reset_handler(void)
{
	const uint32_t app_msp = *(const uint32_t *)APP_BASE;
	const uint32_t app_reset = *(const uint32_t *)(APP_BASE + 4u);

	__asm volatile("cpsid i");

	SYST_CSR = 0;
	for(unsigned int i = 0; i < 8; i++)
	{
		NVIC_ICER[i] = 0xFFFFFFFFu;
		NVIC_ICPR[i] = 0xFFFFFFFFu;
	}

	if((app_msp < SRAM_BASE) || (app_msp > STACK_TOP) || ((app_reset & 1u) == 0))
	{
		hang();
	}

#if BOOTSTUB_UART_BANNER
	uart0_banner();
#endif
	microbit_buttons_init();

	SCB_VTOR = APP_BASE;
	__asm volatile("dsb");
	__asm volatile("isb");

	__asm volatile("msr msp, %0\n"
	               "bx %1\n"
	               :
	               : "r"(app_msp), "r"(app_reset)
	               : "memory");

	__builtin_unreachable();
}
