#include "main.h"
#include "spi.h"
#include "gpio.h"
#include "sk6812.h"
#include "key.h"
#include "i2c_ex.h"
#include "flash.h"

// extern unsigned SEGGER_RTT_Write(unsigned BufferIndex, const void* pBuffer,
// unsigned NumBytes);
extern int SEGGER_RTT_printf(unsigned BufferIndex, const char *sFormat, ...);
volatile uint32_t rgb_color           = 0;
volatile uint8_t flag_rgb_color       = 0;
uint8_t g_scale_set[4]                = {0, 0, 255, 0};
volatile uint16_t flag_scale_set_0    = 0;
volatile uint16_t flag_scale_set_load = 0;
volatile uint16_t flag_start_cal      = 0;
volatile uint16_t g_scale_set_0       = 0;
volatile uint16_t g_scale_set_load    = 255;
volatile uint32_t g_adc_0             = 0;
volatile uint32_t g_adc_load          = 0;

#define abs(x) (((x) > 0) ? (x) : (-x))

#define log(format...)

void Sk6812Blink(uint32_t color, uint32_t time) {
    static uint32_t blink_time_start = 0;
    static uint32_t now_state        = 0;
    uint32_t ticks                   = HAL_GetTick();
    if (time == 0) {
        Sk6812SetColor(0, color);
        Sk6812Show();
    } else if (ticks - blink_time_start > time) {
        color     = now_state ? color : 0;
        now_state = 1 - now_state;
        Sk6812SetColor(0, color);
        Sk6812Show();
        blink_time_start = ticks;
    }
}

void rainbowLED(uint32_t time) {
#define LEDSPEED 0xf
    static uint8_t red;
    static uint8_t green;
    static uint8_t blue;
    static uint32_t tempColor;
    static uint32_t oldTime;
    static uint8_t t;
    if (oldTime == 0x00) {
        oldTime   = time;
        red       = 0x00;
        green     = 0x00;
        blue      = 0x00;
        tempColor = 0x00;

        t = 0x00;
    }
    if (time > oldTime + 3 && t <= (LEDSPEED * 6)) {
        oldTime = time;
        t++;
        if (t < LEDSPEED) {
            red  = 0xff;
            blue = 0x0;
            green += (0xff / LEDSPEED);
        }
        if (t > LEDSPEED && t < LEDSPEED * 2) {
            red -= (0xff / LEDSPEED);
            blue  = 0x0;
            green = 0xff;
        }
        if (t > LEDSPEED * 2 && t < LEDSPEED * 3) {
            red = 0x0;
            blue += (0xff / LEDSPEED);
            green = 0xff;
        }
        if (t > LEDSPEED * 3 && t < LEDSPEED * 4) {
            red  = 0x0;
            blue = 0xff;
            green -= (0xff / LEDSPEED);
        }
        if (t > LEDSPEED * 4 && t < LEDSPEED * 5) {
            red += (0xff / LEDSPEED);
            blue  = 0xff;
            green = 0x0;
        }
        if (t > LEDSPEED * 5 && t < LEDSPEED * 6) {
            red -= (0xff / LEDSPEED);
            blue -= (0xff / LEDSPEED);
            green = 0x0;
        }
        if (t == LEDSPEED * 6) {
            red   = 0x0;
            blue  = 0x0;
            green = 0x0;
        }
        tempColor = (red) | (green << 8) | (blue << 16);
        Sk6812SetColor(0, tempColor);
        Sk6812Show();
        // HAL_Delay(50);
        tempColor = 0x00;
    }
}

#define HX711_getDataValue() (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3))
#define HX711_setClkValue(x) \
    (HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, (GPIO_PinState)x))

uint32_t HX711_readData(void) {
    uint8_t time_count      = 0;
    volatile uint8_t unused = 0;
    uint32_t count          = 0;

    // wait hx711 cover finish
    while (HX711_getDataValue()) {
        HAL_Delay(1);
        time_count++;
        if (time_count == 200) {
            log("adc wait for a timeout\r\n");
            return 0;
        }
    }

    for (uint8_t i = 0; i < 24; i++) {
        HX711_setClkValue(1);
        count = count << 1;
        HX711_setClkValue(0);

        count = count + HX711_getDataValue();
    }

    HX711_setClkValue(1);
    unused = unused + 1;
    HX711_setClkValue(0);

    count = count ^ 0x800000;
    return count;
}

uint32_t getOffset(void) {
    return HX711_readData();
}

void calScale(void) {
    uint16_t diff_load = g_scale_set_load > g_scale_set_0
                             ? (g_scale_set_load - g_scale_set_0)
                             : (g_scale_set_0 - g_scale_set_load);
    uint32_t diff_adc =
        g_adc_0 > g_adc_load ? (g_adc_0 - g_adc_load) : (g_adc_load - g_adc_0);
    if (diff_adc == 0) return;
    if (diff_load == 0) return;
    g_scale = (double)diff_adc / (double)diff_load;
    if (g_scale > 100.0) g_scale = g_scale / 100.0;
    if (readPackedMessageFromFlash(flash_arg, 8)) {
        flash_arg[2] = diff_load & 0xff;
        flash_arg[3] = (diff_load >> 8) & 0xff;

        flash_arg[4] = diff_adc & 0xff;
        flash_arg[5] = (diff_adc >> 8) & 0xff;
        flash_arg[6] = (diff_adc >> 16) & 0xff;
        flash_arg[7] = (diff_adc >> 24) & 0xff;

        write_to_flash();
    }
}

uint32_t getWeight(uint32_t offset, volatile uint32_t *primitiveVal,
                   volatile double scale) {
#define FILTERSIZE 10
    static int32_t filter[FILTERSIZE] = {0};
    static uint8_t pfilter            = 0;
    uint32_t data;
    int32_t temp;
    int32_t weightVal;

    data          = HX711_readData();
    *primitiveVal = data;

    if (flag_scale_set_0) {
        flag_scale_set_0 = 0;
        g_adc_0          = HX711_readData();
    }
    if (flag_scale_set_load) {
        flag_scale_set_load = 0;
        g_adc_load          = HX711_readData();
        flag_start_cal      = 1;
    }
    if (flag_start_cal) {
        flag_start_cal = 0;
        calScale();
    }

    filter[pfilter] = data;
    if (pfilter < FILTERSIZE) {
        pfilter++;
    } else {
        pfilter = 0;
    }

    log("val : %d.%d\r\n", (int)(temp / 1.0), abs(((int)(temp * 100)) % 100));

    //	temp = 0;
    //	for (int x = 0; x < FILTERSIZE; x++)
    //	{
    //		temp += 0.1 * filter[x];
    //	}

    temp = (int)(offset - data);

    weightVal = temp / g_scale;  // 1.560;

    return weightVal;
}

uint8_t interval100ms(void) {
    static uint32_t lastTime = 0;
    uint32_t time            = 0;
    time                     = HAL_GetTick();
    if (time > lastTime + 100) {
        lastTime = time;
        return 1;
    } else {
        return 0;
    }
}

#define GET_KEY_VALUE() ((GPIOA->IDR & 0x04) == 0x04)

volatile struct reg32Type {
    uint32_t primitive;
    uint32_t weight;
    uint32_t offset;
    uint32_t scale;
} reg32addr10 = {0, 0, 0, 156};

volatile struct reg32Type2 { uint32_t regSk6812Color; } reg32addr50 = {0x00};

volatile struct reg8Type {
    uint8_t keyPressed;
    uint8_t keyLongPressed;
    uint8_t ketState;
    uint8_t keyTare;
    uint8_t regTare;
    uint8_t ledNotDisplayWeight;
} reg8addr20 = {0, 0, 0, 0, 0, 0};

void app_main(void) {
    Sk6812Init(1);

    log("date:%s  time:%s\r\n", __DATE__, __TIME__);
    I2CAddReg(0x10, (uint8_t *)&reg32addr10, sizeof(struct reg32Type), 32);
    I2CAddReg(0x20, (uint8_t *)&reg8addr20, sizeof(struct reg8Type), 8);
    I2CAddReg(0x50, (uint8_t *)&reg32addr50, sizeof(struct reg32Type2), 32);
    I2CInit();

    reg32addr10.offset = getOffset();
    for (;;) {
        rainbowLED(HAL_GetTick());

        reg8addr20.ketState = GET_KEY_VALUE();
        switch (KeyUpdate()) {
            case kReleased:
                reg8addr20.keyPressed++;
                if (reg8addr20.keyPressed != 0x00) {
                    if (reg8addr20.keyTare) {
                        reg32addr10.offset = getOffset();
                    }
                }
                break;
            case kLongPressed:
                reg8addr20.keyLongPressed++;
                break;
            default:;
        }

        if (reg8addr20.regTare) {
            // if tare register is not 0 ,reread offset
            reg32addr10.offset = getOffset();
            reg8addr20.regTare = 0x0;
        }

        if (interval100ms()) {
            // set weight register
            reg32addr10.weight =
                getWeight(reg32addr10.offset, &reg32addr10.primitive, g_scale);

            if (reg8addr20.ledNotDisplayWeight == 0x01) {
                int16_t temp = (reg32addr10.weight / 100) / 256;
                if (temp < 0xff) {
                    if (temp >= 0) {
                        Sk6812SetColor(0, temp | temp << 8 | temp << 16);
                    } else {
                        Sk6812SetColor(0, 0x0);
                    }
                } else {
                    Sk6812SetColor(0, 0xffffff);
                }
                Sk6812Show();
            }
        }

        if (flag_rgb_color) {
            flag_rgb_color = 0;
            Sk6812SetColor(0, rgb_color);
            Sk6812Show();
            if (reg8addr20.ledNotDisplayWeight == 0x1) {
                reg8addr20.ledNotDisplayWeight = 0x0;
            }
        }
    }
}
