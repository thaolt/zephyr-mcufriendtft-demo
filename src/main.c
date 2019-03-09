#include <zephyr.h>
#include <gpio.h>
#include <misc/printk.h>
#include <shell/shell.h>
#include <shell/shell_uart.h>
#include <device.h>

struct device *gpioa;
struct device *gpiob;
struct device *gpioc;

#define GPIOC1 gpioc,1
#define GPIOB0 gpiob,0
#define GPIOA4 gpioa,4
#define GPIOA0 gpioa,0
#define GPIOA1 gpioa,1

#define GPIOA9 gpioa,9
#define GPIOA8 gpioa,8
#define GPIOC7 gpioc,7
#define GPIOA10 gpioa,10
#define GPIOB3 gpiob,3
#define GPIOB5 gpiob,5
#define GPIOB4 gpiob,4
#define GPIOB10 gpiob,10
#define GPIOB8 gpiob,8

#define LCD_RST GPIOC1
#define LCD_CS GPIOB0
#define LCD_RS GPIOA4
#define LCD_WR GPIOA1
#define LCD_RD GPIOA0

#define LCD_D0 GPIOA9
#define LCD_D1 GPIOC7
#define LCD_D2 GPIOA10
#define LCD_D3 GPIOB3
#define LCD_D4 GPIOB5
#define LCD_D5 GPIOB4
#define LCD_D6 GPIOB10
#define LCD_D7 GPIOA8

#define HIGH 1
#define LOW 0

#define OUTPUT GPIO_DIR_OUT
#define INPUT GPIO_DIR_IN

#define pinMode gpio_pin_configure
#define digitalWrite gpio_pin_write

uint8_t unlock_1520[]     = { (0xB0), 2, 0x00, 0x00 };
uint8_t unlock_1526[]     = { (0xB0), 2, 0x3F, 0x3F };
uint8_t unlock_8357[]     = { (0xB9), 3, 0xFF, 0x83, 0x57};
uint8_t unlock_5310[] = { (0xED), 2, 0x01, 0xFE};
uint8_t d5310_0_in[] = { (0xEE), 2, 0xDE, 0x21}; //enter CMD3 8bit args
//uint8_t d5310_1[] = { (0x??), 2, 0xDE, 0x21};
uint8_t d5310_1_in[] = { (0xBF), 1, 0xAA}; //enter page#1 16bit args
uint8_t d5310_1_out[] = { (0x00), 1, 0xAA}; //leave page#1 16bit args
uint8_t d1526nvm[]  = { (0xE2), 1, 0x3F};
uint8_t *unlock     = NULL;
uint8_t *page_N     = NULL;

void printhex(uint8_t val)
{
    printk("%02x", val);
}

u32_t digitalRead(struct device *dev, u32_t index)
{
  u32_t value = 0;
  u32_t rc = gpio_pin_read(dev, index, &value);
  return value;
}

uint16_t lcdRead8()
{
    uint16_t result = digitalRead(LCD_D7);
    result <<= 1;
    result |= digitalRead(LCD_D6);
    result <<= 1;
    result |= digitalRead(LCD_D5);
    result <<= 1;
    result |= digitalRead(LCD_D4);
    result <<= 1;
    result |= digitalRead(LCD_D3);
    result <<= 1;
    result |= digitalRead(LCD_D2);
    result <<= 1;
    result |= digitalRead(LCD_D1);
    result <<= 1;
    result |= digitalRead(LCD_D0);

    return result;
}

uint8_t lcdReadData8()
{
    uint8_t result;
    lcdSetReadDir();
    digitalWrite(LCD_CS, LOW);
    digitalWrite(LCD_RS, HIGH);
    digitalWrite(LCD_RD, HIGH);
    digitalWrite(LCD_WR, HIGH);

    digitalWrite(LCD_RD, LOW);
    k_busy_wait(10);
    result = lcdRead8();
    digitalWrite(LCD_RD, HIGH);

    k_busy_wait(10);

    return result;
}

void readReg(uint16_t reg, uint8_t n, const char *msg)
{
    uint8_t val8;
    lcdReset();
    lcdSetWriteDir();
    if (unlock) pushCommand(unlock[0], unlock + 2, unlock[1]);
    if (page_N) pushCommand(page_N[0], page_N + 2, page_N[1]);
    printk("reg(0x");
    printhex(reg >> 8);
    printhex(reg);
    printk(")");
    lcdWriteCommand(reg);
    lcdSetReadDir();
    while (n--) {
        val8 = lcdReadData8();
        printk(" ");
        printhex(val8);
    }
    lcdSetWriteDir();
    digitalWrite(LCD_CS, HIGH);

    printk("\t");
    printk("%s\n",msg);
}

void lcdInit()
{
    pinMode(LCD_CS, OUTPUT);
    digitalWrite(LCD_CS, HIGH);
    pinMode(LCD_RS, OUTPUT);
    digitalWrite(LCD_RS, HIGH);
    pinMode(LCD_WR, OUTPUT);
    digitalWrite(LCD_WR, HIGH);
    pinMode(LCD_RD, OUTPUT);
    digitalWrite(LCD_RD, HIGH);
    pinMode(LCD_RST, OUTPUT);
    digitalWrite(LCD_RST, HIGH);
}

void lcdReset()
{
    digitalWrite(LCD_RST, LOW);
    k_sleep(2);
    digitalWrite(LCD_RST, HIGH);
    k_sleep(10);             //allow controller to re-start
}

void lcdWrite8(uint16_t data)
{
    digitalWrite(LCD_D0, data & 1);
    digitalWrite(LCD_D1, (data & 2) >> 1);
    digitalWrite(LCD_D2, (data & 4) >> 2);
    digitalWrite(LCD_D3, (data & 8) >> 3);
    digitalWrite(LCD_D4, (data & 16) >> 4);
    digitalWrite(LCD_D5, (data & 32) >> 5);
    digitalWrite(LCD_D6, (data & 64) >> 6);
    digitalWrite(LCD_D7, (data & 128) >> 7);
}

void lcdSetWriteDir()
{
    uint8_t mode = OUTPUT;
    pinMode(LCD_D0, mode);
    pinMode(LCD_D1, mode);
    pinMode(LCD_D2, mode);
    pinMode(LCD_D3, mode);
    pinMode(LCD_D4, mode);
    pinMode(LCD_D5, mode);
    pinMode(LCD_D6, mode);
    pinMode(LCD_D7, mode);
}


void lcdSetReadDir()
{
    uint8_t mode = INPUT;
    pinMode(LCD_D0, mode);
    pinMode(LCD_D1, mode);
    pinMode(LCD_D2, mode);
    pinMode(LCD_D3, mode);
    pinMode(LCD_D4, mode);
    pinMode(LCD_D5, mode);
    pinMode(LCD_D6, mode);
    pinMode(LCD_D7, mode);
}

void lcdWriteData(uint16_t data)
{
    lcdSetWriteDir();
    digitalWrite(LCD_CS, LOW);
    digitalWrite(LCD_RS, HIGH);
    digitalWrite(LCD_RD, HIGH);
    digitalWrite(LCD_WR, HIGH);

    lcdWrite8(data >> 8);

    digitalWrite(LCD_WR, LOW);
    k_busy_wait(10);
    digitalWrite(LCD_WR, HIGH);

    lcdWrite8(data);

    digitalWrite(LCD_WR, LOW);
    k_busy_wait(10);
    digitalWrite(LCD_WR, HIGH);

    digitalWrite(LCD_CS, HIGH);
}

void lcdWriteCommand(uint16_t command)
{
    lcdSetWriteDir();
    digitalWrite(LCD_CS, LOW);
    digitalWrite(LCD_RS, LOW);
    digitalWrite(LCD_RD, HIGH);
    digitalWrite(LCD_WR, HIGH);
    lcdWrite8(command >> 8);
    digitalWrite(LCD_WR, LOW);
    k_busy_wait(10);
    digitalWrite(LCD_WR, HIGH);
    lcdWrite8(command);
    digitalWrite(LCD_WR, LOW);
    k_busy_wait(10);
    digitalWrite(LCD_WR, HIGH);
    //    digitalWrite(LCD_CS, HIGH);
}

uint16_t lcdReadData16()
{
    uint16_t result;
    result = lcdReadData8() << 8;
    result |= lcdReadData8();
    return result;
}


void lcdWriteRegister(uint16_t addr, uint16_t data)
{
    lcdWriteCommand(addr);
    lcdWriteData(data);
}

void pushCommand(uint8_t command, uint8_t *block, int8_t n)
{
    lcdSetWriteDir();
    digitalWrite(LCD_CS, LOW);
    digitalWrite(LCD_RS, LOW);
    digitalWrite(LCD_RD, HIGH);
    digitalWrite(LCD_WR, HIGH);
    lcdWrite8(command);
    digitalWrite(LCD_WR, LOW);
    k_busy_wait(10);
    digitalWrite(LCD_WR, HIGH);
    digitalWrite(LCD_RS, HIGH);
    while (n--) {
        lcdWrite8(*block++);
        digitalWrite(LCD_WR, LOW);
        k_busy_wait(10);
        digitalWrite(LCD_WR, HIGH);
    }
    digitalWrite(LCD_CS, HIGH);
}


static void read_regs(char *title)
{
    readReg(0x00, 2, "ID: ILI9320, ILI9325, ILI9335, ...");
    readReg(0x04, 4, "Manufacturer ID");
    readReg(0x09, 5, "Status Register");
    readReg(0x0A, 2, "Get Powsr Mode");
    readReg(0x0C, 2, "Get Pixel Format");
    readReg(0x30, 5, "PTLAR");
    readReg(0x33, 7, "VSCRLDEF");
    readReg(0x61, 2, "RDID1 HX8347-G");
    readReg(0x62, 2, "RDID2 HX8347-G");
    readReg(0x63, 2, "RDID3 HX8347-G");
    readReg(0x64, 2, "RDID1 HX8347-A");
    readReg(0x65, 2, "RDID2 HX8347-A");
    readReg(0x66, 2, "RDID3 HX8347-A");
    readReg(0x67, 2, "RDID Himax HX8347-A");
    readReg(0x70, 2, "Panel Himax HX8347-A");
    readReg(0xA1, 5, "RD_DDB SSD1963");
    readReg(0xB0, 2, "RGB Interface Signal Control");
    readReg(0xB3, 5, "Frame Memory");
    readReg(0xB4, 2, "Frame Mode");
    readReg(0xB6, 5, "Display Control");
    readReg(0xB7, 2, "Entry Mode Set");
    readReg(0xBF, 6, "ILI9481, HX8357-B");
    readReg(0xC0, 9, "Panel Control");
    readReg(0xC1, 4, "Display Timing");
    readReg(0xC5, 2, "Frame Rate");
    readReg(0xC8, 13, "GAMMA");
    readReg(0xCC, 2, "Panel Control");
    readReg(0xD0, 4, "Power Control");
    readReg(0xD1, 4, "VCOM Control");
    readReg(0xD2, 3, "Power Normal");
    readReg(0xD3, 4, "ILI9341, ILI9488");
    readReg(0xD4, 4, "Novatek");
    readReg(0xDA, 2, "RDID1");
    readReg(0xDB, 2, "RDID2");
    readReg(0xDC, 2, "RDID3");
    readReg(0xE0, 16, "GAMMA-P");
    readReg(0xE1, 16, "GAMMA-N");
    readReg(0xEF, 6, "ILI9327");
    readReg(0xF2, 12, "Adjust Control 2");
    readReg(0xF6, 4, "Interface Control");
}

static int cmd_detectlcd(const struct shell *shell, size_t argc, char **argv)
{
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);

  shell_print(shell, "Detecting LCD Shield IC...");
  lcdInit();
  lcdReset();
  read_regs("diagnose any controller");

  return 0;
}

SHELL_CMD_ARG_REGISTER(detectlcd, NULL, "detect lcd shield IC", cmd_detectlcd, 1, 0);

void main(void)
{
  gpioa = device_get_binding("GPIOA");
  gpiob = device_get_binding("GPIOB");
  gpioc = device_get_binding("GPIOC");
	printk("Hello Shell Demo App v0.0.1, running on %s\n", CONFIG_BOARD);
}

