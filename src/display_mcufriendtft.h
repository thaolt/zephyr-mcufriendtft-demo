#include <string.h>

#include <display.h>

#include <zephyr.h>
#include <gpio.h>
#include <misc/printk.h>
#include <shell/shell.h>
#include <shell/shell_uart.h>
#include <device.h>



static int cmd_detectlcd(const struct shell *shell, size_t argc, char **argv);
u32_t digitalRead(struct device *dev, u32_t index);
void lcdInit();
uint16_t lcdRead8();
uint16_t lcdReadData16();
uint8_t lcdReadData8();
void lcdReset();
void lcdSetReadDir();
void lcdSetWriteDir();
void lcdWrite8(uint16_t data);
void lcdWriteCommand(uint16_t command);
void lcdWriteData(uint16_t data);
void lcdWriteRegister(uint16_t addr, uint16_t data);
void printhex(uint8_t val);
void pushCommand(uint8_t command, uint8_t *block, int8_t n);
void readReg(uint16_t reg, uint8_t n, const char *msg);
static void read_regs(char *title);

