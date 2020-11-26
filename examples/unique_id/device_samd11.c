#include <stdint.h>
#include <stdbool.h>

#include "samd11.h"
#include "device_samd11.h"

/* ------ Function prototypes ------ */

static void _setup_clk(void);
static void _setup_tc1(void);
static void _setup_adc(void);
static void _setup_uart(void);
static void _setup_evsys(void);
static void _setup_dmac(void);

//-----------------------------------------------------------------------------
static void _setup_clk(void) {
    // REVISIT setup 48 MHz DFLL for core?

    /* Setup internal 32.768 kHz oscillator */
    SYSCTRL_OSC32K_Type sysctrl_osc32k = {
        .bit.ENABLE = true,         /* Enable oscillator */
        // REVISIT EN32K needs to be true. Datasheet has no information why.
        .bit.EN32K = true,          /* Enable 32 kHz output */
        .bit.EN1K = false,          /* Enable 1 kHz output */
        .bit.RUNSTDBY = true,       /* Run in standby */
        .bit.ONDEMAND = false,      /* Run on demand */
        .bit.STARTUP = 0x0,         /* Startup time */
        .bit.WRTLOCK = false,       /* Enable write lock */
        .bit.CALIB = 0x3F           /* (Reset) Calibration value */
    };
    SYSCTRL->OSC32K.reg = sysctrl_osc32k.reg;
    while (!SYSCTRL->PCLKSR.bit.OSC32KRDY);

    /* Use calibration value from NVM (Table 9-4). */
    // REVISIT datasheet implies this can only be done after OSC32K is ready.
    // Take some measurements to determine if this is actually true.
    uint32_t osc32k_calib = (*FACTORY_CALIB_HIGH >> 6) & 0x7F;
    SYSCTRL->OSC32K.bit.CALIB = osc32k_calib;

    /* Connect OSC32K to genenerator 3 */
    GCLK_GENCTRL_Type genctrl = {
        .bit.ID = 3,                    /* GCLK generator selection */
        .bit.SRC = GCLK_SOURCE_OSC32K,  /* Source select */
        .bit.GENEN = true,              /* Generator enable */
        .bit.IDC = false,               /* Improve duty cycle */
        .bit.OOV = false,               /* Output Off value */
        .bit.OE = false,                /* Output enable */
        .bit.DIVSEL = false,            /* Divide selection */
        .bit.RUNSTDBY = true            /* Run in standby */
    };
    GCLK->GENCTRL.reg = genctrl.reg;
    while (GCLK->STATUS.bit.SYNCBUSY);

    /* Set OSC8M to 8 MHz */
    SYSCTRL->OSC8M.bit.PRESC = SYSCTRL_OSC8M_PRESC_0_Val;
    while (!SYSCTRL->PCLKSR.bit.OSC8MRDY);

    /* Connect OSC8M to generator 4 */
    genctrl.bit.ID = 4;
    genctrl.bit.SRC = GCLK_SOURCE_OSC8M;
    genctrl.bit.RUNSTDBY = false;
    GCLK->GENCTRL.reg = genctrl.reg;
    while (GCLK->STATUS.bit.SYNCBUSY);
}

//-----------------------------------------------------------------------------
static void _setup_tc1(void) {

    /* Enable APB bus clock */
    PM->APBCMASK.reg |= PM_APBCMASK_TC1;

    /* Setup GCLK driver - generator 3, driven by OSC32K */
    GCLK_CLKCTRL_Type clkctrl = {
        .bit.ID = TC1_GCLK_ID,  /* Generic clock selection ID */
        .bit.GEN = 3,           /* Generic clock generator indux */
        .bit.CLKEN = true,      /* Clock enable */
        .bit.WRTLOCK = false    /* Write lock */
    };
    GCLK->CLKCTRL.reg = clkctrl.reg;

    /* Setup TC */
    TC_CTRLA_Type crtla = {
        .bit.SWRST = false,                             /* Software reset */
        .bit.ENABLE = false,                            /* Enable */
        .bit.MODE = TC_CTRLA_MODE_COUNT16_Val,          /* TC mode */
        .bit.WAVEGEN = TC_CTRLA_WAVEGEN_MFRQ_Val,       /* Waveform generation operation */
        .bit.PRESCALER = TC_CTRLA_PRESCALER_DIV1_Val,   /* Prescalar */
        .bit.RUNSTDBY = true,                           /* Run in standby */
        .bit.PRESCSYNC = TC_CTRLA_PRESCSYNC_PRESC_Val   /* Prescalar/counter sync */
    };
    TC1->COUNT16.CTRLA.reg = crtla.reg;
}

//-----------------------------------------------------------------------------
static void _setup_adc(void) {

    /* APB bus clock is enabled by default (Table 15-1) */
    /* Setup GCLK driver - generator 4 */
    GCLK_CLKCTRL_Type clkctrl = {
        .bit.ID = ADC_GCLK_ID,  /* Generic clock selection ID */
        .bit.GEN = 4,           /* Generic clock generator indux */
        .bit.CLKEN = true,      /* Clock enable */
        .bit.WRTLOCK = false    /* Write lock */
    };
    GCLK->CLKCTRL.reg = clkctrl.reg;

    /* Set up reference - external VREFA*/
    ADC_REFCTRL_Type refctrl = {
        .bit.REFSEL = ADC_REFCTRL_REFSEL_AREFA_Val, /* Reference Selection */
        .bit.REFCOMP = true                         /* Reference Buffer Offset Compensation Enable */
    };
    ADC->REFCTRL.reg = refctrl.reg;

    /* Set up conversion mode - differential, free running */
    ADC_CTRLB_Type ctrlb = {
        .bit.DIFFMODE = true,                           /* Differential mode */
        .bit.LEFTADJ = false,                           /* Left-adjusted result */
        .bit.FREERUN = true,                            /* Free running mode */
        .bit.CORREN = false,                            /* Digital correction logic enabled */
        .bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val,       /* Conversion result resolution */
        .bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV8_Val   /* Prescalar configuration */
    };
    ADC->CTRLB.reg = ctrlb.reg;

    /* Set up input control */
    ADC_INPUTCTRL_Type inputctrl = {
        .bit.MUXPOS = ADC_INPUTCTRL_MUXPOS_PIN2_Val,    /* Positive mux input selection */
        .bit.MUXNEG = ADC_INPUTCTRL_MUXNEG_PIN0_Val,    /* Negative mux input selection */
        .bit.INPUTSCAN = 4,                             /* Number of input channels included in scan */
        .bit.INPUTOFFSET = 0,                           /* Positive mux setting offset */
        .bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val           /* Gain factor selection */
    };
    ADC->INPUTCTRL.reg = inputctrl.reg;

    /* Set up event control */
    ADC_EVCTRL_Type evctrl = {
        // REVISIT start conversion on TC overflow event, rather than interrupt
        .bit.STARTEI = false,   /* Start conversion event in */
        .bit.SYNCEI = false,    /* Synchronisation event in */
        .bit.RESRDYEO = true,   /* Result ready event out */
        .bit.WINMONEO = false   /* Window monitor event out */
    };
    ADC->EVCTRL.reg = evctrl.reg;

    /* Load factory calibration values - Table 9-4 */
    uint32_t adc_calib_lin = (*FACTORY_CALIB_LOW >> 27);
    adc_calib_lin |= ((*FACTORY_CALIB_HIGH & 0x7) << 5);
    uint32_t adc_calib_bias = (*FACTORY_CALIB_HIGH >> 3) & 0x7;
    ADC_CALIB_Type calib = {
        .bit.LINEARITY_CAL = adc_calib_lin, /* Linearity calibration value */
        .bit.BIAS_CAL = adc_calib_bias      /* Bias calibration value */
    };
    ADC->CALIB.reg = calib.reg;
}


//-----------------------------------------------------------------------------
static void _setup_uart(void) {

    /* Set up pins */
    // HAL_GPIO_UART_TX_out();
    // HAL_GPIO_UART_TX_pmuxen(PORT_PMUX_PMUXE_D_Val);  // SERCOM alt mapping
    // HAL_GPIO_UART_RX_in();
    // HAL_GPIO_UART_RX_pmuxen(PORT_PMUX_PMUXE_D_Val);

    /* Activate APB clock */
    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0;

    /* Setup GCLK driver - generator 4 */
    GCLK_CLKCTRL_Type clkctrl = {
        .bit.ID = SERCOM0_GCLK_ID_CORE, /* Generic clock selection ID */
        .bit.GEN = 4,                   /* Generic clock generator indux */
        .bit.CLKEN = true,              /* Clock enable */
        .bit.WRTLOCK = false            /* Write lock */
    };
    GCLK->CLKCTRL.reg = clkctrl.reg;

    SERCOM_USART_CTRLA_Type ctrla = {
        .bit.SWRST = false,                                 /* Software reset */
        .bit.ENABLE = false,                                /* Enable */
        .bit.MODE = SERCOM_USART_CTRLA_MODE_USART_INT_CLK_Val,    /* Operating mode */
        .bit.RUNSTDBY = false,                              /* Run during standby */
        .bit.IBON = false,                                  /* Immediate buffer overflow notification */
        .bit.SAMPR = 0,                                     /* Sample */
        .bit.TXPO = 1,                                      /* Transmit data pinout */ // SERCOM[2]
        .bit.RXPO = 3,                                      /* Receive data pinout */ // SERCOM[3]
        .bit.SAMPA = 0,                                     /* Sample adjustment */
        .bit.FORM = 0,                                      /* Frame format */ // USART frame
        .bit.CMODE = 0,                                     /* Communcation mode */
        .bit.CPOL = 0,                                      /* Clock polarity */
        .bit.DORD = 1                                       /* Data order */ // LSB first
    };
    SERCOM0->USART.CTRLA.reg = ctrla.reg;
}

static void _setup_evsys(void) {

    /* Enable APB clock */
    PM->APBCMASK.reg |= PM_APBCMASK_EVSYS;

    /* Set up TC overflow -> ADC sample event */
    /* CHANNEL 1 */
    EVSYS_CHANNEL_Type channel = {
        .bit.CHANNEL = EV_CHAN_TC_ADC,                      /* Channel selection */
        .bit.SWEVT = 0,                                     /* Software event */
        .bit.EVGEN = 0x20,                                  /* Event generator selection, see Table 23-4 */
        .bit.PATH = EVSYS_CHANNEL_PATH_RESYNCHRONIZED_Val,  /* Path selection */
        .bit.EDGSEL = EVSYS_CHANNEL_EDGSEL_RISING_EDGE_Val  /* Edge detection selection */
    };
    EVSYS->CHANNEL.reg = channel.reg;

    EVSYS_USER_Type user = {
        .bit.USER = 0x0C,                   /* User multiplexor selection, Table 23-6 */
        .bit.CHANNEL = EV_CHAN_TC_ADC + 1,  /* Channel event selection, Table 23-5 (CHANNEL - 1) */
    };
    EVSYS->USER.reg = user.reg;

    /* Set up ADC result ready -> DMA beat transfer event */
    /* CHANNEL 2 */
    channel.bit.CHANNEL = EV_CHAN_ADC_DMA;  /* Channel selection */
    channel.bit.EVGEN = 0x25;               /* Event generator selection, Table 23-4 */
    EVSYS->CHANNEL.reg = channel.reg;

    user.bit.USER = 0x00;                   /* User multiplexor selection, Table 23-6 */
    user.bit.CHANNEL = EV_CHAN_ADC_DMA + 1; /* Channel event selection */
    EVSYS->USER.reg = user.reg;
}

static void _setup_dmac(void) {
    // REVISIT
    return;
}

void setup_device(void) {

    _setup_clk();
    _setup_tc1();
    _setup_adc();
    _setup_uart();
    _setup_evsys();
    _setup_dmac();

}
