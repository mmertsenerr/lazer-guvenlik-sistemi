#include "stm32g431xx.h"
#include <stdint.h>

/*
    ============================================================
    2 EKSEN MOTOR + LDR ALARM SİSTEMİ
    REGISTER SEVİYESİ

    MOTOR SİSTEMİ:
    X ekseni:
    PB0 -> X TB6560 CLK+ / STEP
    PB7 -> X TB6560 CW+  / DIR

    Y ekseni:
    PB6 -> Y TB6560 CLK+ / STEP
    PA8 -> Y TB6560 CW+  / DIR

    Enable pinleri kullanılmıyor.

    Limit switchler NO bağlı:
    X_MIN -> PF0
    X_MAX -> PF1
    Y_MIN -> PB5
    Y_MAX -> PB4

    Limit switch COM uçları -> GND

    Motor anahtarı:
    PA12 -> Anahtar -> GND

    Anahtar ON  -> PA12 LOW  -> sistem çalışır
    Anahtar OFF -> PA12 HIGH -> sistem durur

    ------------------------------------------------------------

    ALARM SİSTEMİ:
    LDR Bölge 1 -> PA0 / ADC1_IN1
    LDR Bölge 2 -> PA1 / ADC1_IN2
    LDR Bölge 3 -> PA3 / ADC1_IN4

    Buzzer -> PB8

    Alarm sıfırlama basmalı buton:
    PA11 -> Buton -> GND

    LDR bağlantısı:
    3.3V -> LDR grubu -> ADC pini -> 10k -> GND

    İstenen alarm mantığı:
    Işık kesildiği anda buzzer öter.
    Işık geri gelse bile buzzer susmaz.
    PA11 reset butonuna basınca buzzer susar.
    ============================================================
*/


/* ============================================================
   GENEL MAKRO
   ============================================================ */

#define PIN(n)                  (1U << (n))


/* ============================================================
   X EKSENİ PİNLERİ
   ============================================================ */

#define X_STEP_PIN              PIN(0)      // PB0
#define X_DIR_PIN               PIN(7)      // PB7
#define X_MIN_LIMIT_PIN         PIN(0)      // PF0
#define X_MAX_LIMIT_PIN         PIN(1)      // PF1


/* ============================================================
   Y EKSENİ PİNLERİ
   ============================================================ */

#define Y_STEP_PIN              PIN(6)      // PB6
#define Y_DIR_PIN               PIN(8)      // PA8
#define Y_MIN_LIMIT_PIN         PIN(5)      // PB5
#define Y_MAX_LIMIT_PIN         PIN(4)      // PB4


/* ============================================================
   MOTOR ANAHTARI
   ============================================================ */

#define SWITCH_PIN              PIN(12)     // PA12


/* ============================================================
   LDR / ALARM PİNLERİ
   ============================================================ */

#define LDR1_CHANNEL            1U          // PA0 -> ADC1_IN1
#define LDR2_CHANNEL            2U          // PA1 -> ADC1_IN2
#define LDR3_CHANNEL            4U          // PA3 -> ADC1_IN4

#define BUZZER_PIN              PIN(8)      // PB8
#define ALARM_RESET_PIN         PIN(11)     // PA11


/* ============================================================
   ADC AYARLARI
   ============================================================ */

#define ADC_MAX_VALUE           4095U
#define VREF_MV                 3300U

/*
    Sistem açılırken lazerler LDR'lere normal şekilde vuruyor olmalı.

    Kod açılışta normal ışık değerini öğrenir.
    Eşik = normal_değer * %70 yapılır.

    Işık kesilince ADC değeri eşikten aşağı düşerse alarm oluşur.

    Eğer alarm çok kolay tetiklenirse 60U yap.
    Eğer geç tetiklenirse 80U yap.
*/
#define LDR_THRESHOLD_PERCENT   70U


/* ============================================================
   MOTOR HIZ AYARLARI
   ============================================================ */

#define X_STEP_DELAY_US         50U

#define Y_UP_STEP_DELAY_US      100U
#define Y_DOWN_STEP_DELAY_US    200U

#define STEP_PULSE_HIGH_US      20U


/* ============================================================
   MOTOR GÜVENLİK AYARLARI
   ============================================================ */

#define RELEASE_STEPS           300U
#define SAFETY_MARGIN_STEPS     300U
#define TARGET_WAIT_MS          300U
#define LIMIT_DEBOUNCE_MS       30U


/* ============================================================
   YÖN AYARLARI
   ============================================================ */

#define DIR_MIN                 0U
#define DIR_MAX                 1U

#define X_DIR_TO_MIN_LEVEL      0U
#define X_DIR_TO_MAX_LEVEL      1U

#define Y_DIR_TO_MIN_LEVEL      0U
#define Y_DIR_TO_MAX_LEVEL      1U

/*
    Eğer Y yukarı/aşağı hızları ters çalışırsa bunu 0U yap.
*/
#define Y_DIR_MAX_IS_UP         1U


/* ============================================================
   MOTOR KONUM DEĞİŞKENLERİ
   ============================================================ */

static int32_t xPosition = 0;
static int32_t yPosition = 0;

static int32_t xMaxPosition = 0;
static int32_t yMaxPosition = 0;

static uint8_t xDirection = DIR_MAX;
static uint8_t yDirection = DIR_MAX;

static uint32_t randomSeed = 1234567U;


/* ============================================================
   ALARM DEĞİŞKENLERİ
   ============================================================ */

static volatile uint16_t ldr1_raw = 0;
static volatile uint16_t ldr2_raw = 0;
static volatile uint16_t ldr3_raw = 0;

static volatile uint32_t ldr1_mv = 0;
static volatile uint32_t ldr2_mv = 0;
static volatile uint32_t ldr3_mv = 0;

static uint32_t ldr1_threshold_mv = 1000U;
static uint32_t ldr2_threshold_mv = 1000U;
static uint32_t ldr3_threshold_mv = 1000U;

static uint8_t alarmReady = 0U;
static uint8_t alarmActive = 0U;


/* ============================================================
   FONKSİYON PROTOTİPLERİ
   ============================================================ */

static void alarm_service(void);


/* ============================================================
   GPIO YARDIMCI FONKSİYONLARI
   ============================================================ */

static inline void gpio_set(GPIO_TypeDef *port, uint32_t pin)
{
    port->BSRR = pin;
}

static inline void gpio_reset(GPIO_TypeDef *port, uint32_t pin)
{
    port->BSRR = pin << 16U;
}

static inline uint8_t gpio_read(GPIO_TypeDef *port, uint32_t pin)
{
    return (port->IDR & pin) ? 1U : 0U;
}

static inline void gpio_write(GPIO_TypeDef *port, uint32_t pin, uint8_t value)
{
    if (value)
    {
        gpio_set(port, pin);
    }
    else
    {
        gpio_reset(port, pin);
    }
}


/* ============================================================
   DWT MİKROSANİYE GECİKME
   ============================================================ */

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);

    while ((DWT->CYCCNT - start) < ticks)
    {
        __NOP();
    }
}

static void delay_ms(uint32_t ms)
{
    while (ms--)
    {
        delay_us(1000U);
    }
}

static void delay_ms_with_alarm(uint32_t ms)
{
    while (ms--)
    {
        delay_us(1000U);
        alarm_service();
    }
}


/* ============================================================
   GPIO INIT FONKSİYONLARI
   ============================================================ */

static void gpio_output_init(GPIO_TypeDef *port, uint8_t pinNumber)
{
    uint32_t pos = pinNumber * 2U;

    port->MODER &= ~(3U << pos);
    port->MODER |=  (1U << pos);

    port->OTYPER &= ~PIN(pinNumber);

    port->OSPEEDR &= ~(3U << pos);
    port->OSPEEDR |=  (2U << pos);

    port->PUPDR &= ~(3U << pos);
}

static void gpio_input_pullup_init(GPIO_TypeDef *port, uint8_t pinNumber)
{
    uint32_t pos = pinNumber * 2U;

    port->MODER &= ~(3U << pos);

    port->PUPDR &= ~(3U << pos);
    port->PUPDR |=  (1U << pos);
}

static void gpio_analog_init(GPIO_TypeDef *port, uint8_t pinNumber)
{
    uint32_t pos = pinNumber * 2U;

    port->MODER |= (3U << pos);

    port->PUPDR &= ~(3U << pos);
}

static void gpio_init_all(void)
{
    /*
        GPIOA:
        PA0  -> LDR1 ADC
        PA1  -> LDR2 ADC
        PA3  -> LDR3 ADC
        PA8  -> Y_DIR
        PA11 -> Alarm reset butonu
        PA12 -> Motor anahtarı

        GPIOB:
        PB0 -> X_STEP
        PB4 -> Y_MAX
        PB5 -> Y_MIN
        PB6 -> Y_STEP
        PB7 -> X_DIR
        PB8 -> Buzzer

        GPIOF:
        PF0 -> X_MIN
        PF1 -> X_MAX
    */

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOFEN;

    __DSB();

    /*
        Motor çıkışları.
    */
    gpio_output_init(GPIOB, 0);      // PB0 X_STEP
    gpio_output_init(GPIOB, 7);      // PB7 X_DIR
    gpio_output_init(GPIOB, 6);      // PB6 Y_STEP
    gpio_output_init(GPIOA, 8);      // PA8 Y_DIR

    /*
        Buzzer çıkışı.
    */
    gpio_output_init(GPIOB, 8);      // PB8 BUZZER

    /*
        Limit switch girişleri.
    */
    gpio_input_pullup_init(GPIOF, 0); // PF0 X_MIN
    gpio_input_pullup_init(GPIOF, 1); // PF1 X_MAX
    gpio_input_pullup_init(GPIOB, 5); // PB5 Y_MIN
    gpio_input_pullup_init(GPIOB, 4); // PB4 Y_MAX

    /*
        Motor anahtarı.
    */
    gpio_input_pullup_init(GPIOA, 12); // PA12

    /*
        Alarm reset basmalı buton.
        PA11 -> Buton -> GND
    */
    gpio_input_pullup_init(GPIOA, 11); // PA11

    /*
        LDR analog pinleri.
    */
    gpio_analog_init(GPIOA, 0);        // PA0 ADC1_IN1
    gpio_analog_init(GPIOA, 1);        // PA1 ADC1_IN2
    gpio_analog_init(GPIOA, 3);        // PA3 ADC1_IN4

    /*
        Başlangıçta STEP pinleri ve buzzer LOW.
    */
    gpio_reset(GPIOB, X_STEP_PIN);
    gpio_reset(GPIOB, Y_STEP_PIN);
    gpio_reset(GPIOB, BUZZER_PIN);

    /*
        Başlangıçta DIR pinleri LOW.
    */
    gpio_reset(GPIOB, X_DIR_PIN);
    gpio_reset(GPIOA, Y_DIR_PIN);
}


/* ============================================================
   ADC1 INIT
   ============================================================ */

static void adc1_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;

    __DSB();

    ADC12_COMMON->CCR &= ~ADC_CCR_CKMODE;
    ADC12_COMMON->CCR |= ADC_CCR_CKMODE_0;

    ADC1->CR &= ~ADC_CR_DEEPPWD;

    ADC1->CR |= ADC_CR_ADVREGEN;

    delay_ms(2U);

    ADC1->CR &= ~ADC_CR_ADEN;

    ADC1->CR |= ADC_CR_ADCAL;

    while (ADC1->CR & ADC_CR_ADCAL)
    {
    }

    ADC1->CFGR = 0U;

    /*
        PA0 -> ADC1_IN1
        PA1 -> ADC1_IN2
        PA3 -> ADC1_IN4
    */
    /*
        Uzun sample time.
    */
    ADC1->SMPR1 &= ~((7U << ADC_SMPR1_SMP1_Pos) |
                     (7U << ADC_SMPR1_SMP2_Pos) |
                     (7U << ADC_SMPR1_SMP4_Pos));

    ADC1->SMPR1 |=  ((7U << ADC_SMPR1_SMP1_Pos) |
                     (7U << ADC_SMPR1_SMP2_Pos) |
                     (7U << ADC_SMPR1_SMP4_Pos));

    ADC1->ISR |= ADC_ISR_ADRDY;

    ADC1->CR |= ADC_CR_ADEN;

    while (!(ADC1->ISR & ADC_ISR_ADRDY))
    {
    }
}

static uint16_t adc1_read_channel(uint8_t channel)
{
    ADC1->SQR1 = ((uint32_t)channel << ADC_SQR1_SQ1_Pos);

    ADC1->CR |= ADC_CR_ADSTART;

    while (!(ADC1->ISR & ADC_ISR_EOC))
    {
    }

    return (uint16_t)ADC1->DR;
}

static uint32_t adc_raw_to_mv(uint16_t raw)
{
    return ((uint32_t)raw * VREF_MV) / ADC_MAX_VALUE;
}


/* ============================================================
   ALARM FONKSİYONLARI
   ============================================================ */

static void buzzer_on(void)
{
    gpio_set(GPIOB, BUZZER_PIN);
}

static void buzzer_off(void)
{
    gpio_reset(GPIOB, BUZZER_PIN);
}

static void ldr_read_all(void)
{
    ldr1_raw = adc1_read_channel(LDR1_CHANNEL);
    ldr2_raw = adc1_read_channel(LDR2_CHANNEL);
    ldr3_raw = adc1_read_channel(LDR3_CHANNEL);

    ldr1_mv = adc_raw_to_mv(ldr1_raw);
    ldr2_mv = adc_raw_to_mv(ldr2_raw);
    ldr3_mv = adc_raw_to_mv(ldr3_raw);
}

static void ldr_calibrate_thresholds(void)
{
    uint32_t sum1 = 0U;
    uint32_t sum2 = 0U;
    uint32_t sum3 = 0U;

    uint32_t avg1;
    uint32_t avg2;
    uint32_t avg3;

    uint32_t i;

    /*
        Bu sırada lazer ışığı LDR'lere normal şekilde vuruyor olmalı.
        Normal ışık değerleri öğrenilir.
    */
    for (i = 0U; i < 20U; i++)
    {
        ldr_read_all();

        sum1 += ldr1_mv;
        sum2 += ldr2_mv;
        sum3 += ldr3_mv;

        delay_ms(20U);
    }

    avg1 = sum1 / 20U;
    avg2 = sum2 / 20U;
    avg3 = sum3 / 20U;

    /*
        Işık kesilince ADC düşüyorsa:
        ADC < threshold olduğunda alarm oluşur.
    */
    ldr1_threshold_mv = (avg1 * LDR_THRESHOLD_PERCENT) / 100U;
    ldr2_threshold_mv = (avg2 * LDR_THRESHOLD_PERCENT) / 100U;
    ldr3_threshold_mv = (avg3 * LDR_THRESHOLD_PERCENT) / 100U;

    if (ldr1_threshold_mv < 100U)
    {
        ldr1_threshold_mv = 100U;
    }

    if (ldr2_threshold_mv < 100U)
    {
        ldr2_threshold_mv = 100U;
    }

    if (ldr3_threshold_mv < 100U)
    {
        ldr3_threshold_mv = 100U;
    }

    alarmReady = 1U;
}

static uint8_t beam_broken(void)
{
    /*
        İstenen mantık:
        Işık kesildiği anda buzzer ötsün.

        Bu kodda:
        ADC değeri eşikten aşağı düşerse ışık kesildi kabul edilir.
    */

    if (ldr1_mv < ldr1_threshold_mv)
    {
        return 1U;
    }

    if (ldr2_mv < ldr2_threshold_mv)
    {
        return 1U;
    }

    if (ldr3_mv < ldr3_threshold_mv)
    {
        return 1U;
    }

    return 0U;
}

static uint8_t alarm_reset_button_clicked(void)
{
    static uint8_t lastState = 1U;

    uint8_t currentState = gpio_read(GPIOA, ALARM_RESET_PIN);

    /*
        PA11 pull-up:
        Basılı değil -> HIGH
        Basılı       -> LOW
    */
    if ((lastState == 1U) && (currentState == 0U))
    {
        delay_ms(50U);

        currentState = gpio_read(GPIOA, ALARM_RESET_PIN);

        if (currentState == 0U)
        {
            lastState = currentState;
            return 1U;
        }
    }

    lastState = currentState;

    return 0U;
}

static void alarm_service(void)
{
    uint8_t broken;

    if (!alarmReady)
    {
        return;
    }

    /*
        PA11 reset butonuna basılırsa alarm kapanır.
    */
    if (alarm_reset_button_clicked())
    {
        alarmActive = 0U;
        buzzer_off();
        return;
    }

    /*
        Her çağrıda LDR oku.
        Böylece ışık kesilince daha hızlı tepki verir.
    */
    ldr_read_all();

    broken = beam_broken();

    /*
        Işık kesildiyse alarm aktif olur.
        Işık geri gelse bile alarmActive sıfırlanmaz.
    */
    if (broken)
    {
        alarmActive = 1U;
    }

    if (alarmActive)
    {
        buzzer_on();
    }
    else
    {
        buzzer_off();
    }
}


/* ============================================================
   MOTOR ANAHTARI OKUMA
   ============================================================ */

static uint8_t switch_on(void)
{
    return gpio_read(GPIOA, SWITCH_PIN) == 0U;
}

static void wait_if_switch_off(void)
{
    while (!switch_on())
    {
        gpio_reset(GPIOB, X_STEP_PIN);
        gpio_reset(GPIOB, Y_STEP_PIN);

        alarm_service();

        delay_ms(20U);
    }
}


/* ============================================================
   LİMİT SWITCH OKUMA
   ============================================================ */

static uint8_t x_min_pressed_raw(void)
{
    return gpio_read(GPIOF, X_MIN_LIMIT_PIN) == 0U;
}

static uint8_t x_max_pressed_raw(void)
{
    return gpio_read(GPIOF, X_MAX_LIMIT_PIN) == 0U;
}

static uint8_t y_min_pressed_raw(void)
{
    return gpio_read(GPIOB, Y_MIN_LIMIT_PIN) == 0U;
}

static uint8_t y_max_pressed_raw(void)
{
    return gpio_read(GPIOB, Y_MAX_LIMIT_PIN) == 0U;
}

static uint8_t debounce_limit(uint8_t (*limit_func)(void))
{
    if (!limit_func())
    {
        return 0U;
    }

    delay_ms(LIMIT_DEBOUNCE_MS);

    if (!limit_func())
    {
        return 0U;
    }

    return 1U;
}


/* ============================================================
   MOTOR YÖN AYARLARI
   ============================================================ */

static void x_apply_direction(void)
{
    if (xDirection == DIR_MAX)
    {
        gpio_write(GPIOB, X_DIR_PIN, X_DIR_TO_MAX_LEVEL);
    }
    else
    {
        gpio_write(GPIOB, X_DIR_PIN, X_DIR_TO_MIN_LEVEL);
    }

    delay_us(50U);
}

static void y_apply_direction(void)
{
    if (yDirection == DIR_MAX)
    {
        gpio_write(GPIOA, Y_DIR_PIN, Y_DIR_TO_MAX_LEVEL);
    }
    else
    {
        gpio_write(GPIOA, Y_DIR_PIN, Y_DIR_TO_MIN_LEVEL);
    }

    delay_us(50U);
}

static void x_set_direction(uint8_t dir)
{
    xDirection = dir;
    x_apply_direction();
}

static void y_set_direction(uint8_t dir)
{
    yDirection = dir;
    y_apply_direction();
}


/* ============================================================
   Y HIZ SEÇİMİ
   ============================================================ */

static uint32_t get_y_step_delay_us(void)
{
#if Y_DIR_MAX_IS_UP
    if (yDirection == DIR_MAX)
    {
        return Y_UP_STEP_DELAY_US;
    }
    else
    {
        return Y_DOWN_STEP_DELAY_US;
    }
#else
    if (yDirection == DIR_MAX)
    {
        return Y_DOWN_STEP_DELAY_US;
    }
    else
    {
        return Y_UP_STEP_DELAY_US;
    }
#endif
}


/* ============================================================
   STEP FONKSİYONLARI
   ============================================================ */

static void x_step_raw(void)
{
    alarm_service();

    gpio_set(GPIOB, X_STEP_PIN);
    delay_us(STEP_PULSE_HIGH_US);

    gpio_reset(GPIOB, X_STEP_PIN);
    delay_us(X_STEP_DELAY_US);
}

static void y_step_raw(void)
{
    uint32_t delayValue = get_y_step_delay_us();

    alarm_service();

    gpio_set(GPIOB, Y_STEP_PIN);
    delay_us(STEP_PULSE_HIGH_US);

    gpio_reset(GPIOB, Y_STEP_PIN);
    delay_us(delayValue);
}

static void x_step_with_position(void)
{
    x_step_raw();

    if (xDirection == DIR_MAX)
    {
        xPosition++;
    }
    else
    {
        xPosition--;
    }
}

static void y_step_with_position(void)
{
    y_step_raw();

    if (yDirection == DIR_MAX)
    {
        yPosition++;
    }
    else
    {
        yPosition--;
    }
}


/* ============================================================
   LİMİTTEN UZAKLAŞMA
   ============================================================ */

static uint32_t release_x_min(void)
{
    uint32_t stepCount = 0U;

    x_set_direction(DIR_MAX);

    while (x_min_pressed_raw() && stepCount < RELEASE_STEPS)
    {
        wait_if_switch_off();
        x_step_raw();
        stepCount++;
    }

    return stepCount;
}

static uint32_t release_x_max(void)
{
    uint32_t stepCount = 0U;

    x_set_direction(DIR_MIN);

    while (x_max_pressed_raw() && stepCount < RELEASE_STEPS)
    {
        wait_if_switch_off();
        x_step_raw();
        stepCount++;
    }

    return stepCount;
}

static uint32_t release_y_min(void)
{
    uint32_t stepCount = 0U;

    y_set_direction(DIR_MAX);

    while (y_min_pressed_raw() && stepCount < RELEASE_STEPS)
    {
        wait_if_switch_off();
        y_step_raw();
        stepCount++;
    }

    return stepCount;
}

static uint32_t release_y_max(void)
{
    uint32_t stepCount = 0U;

    y_set_direction(DIR_MIN);

    while (y_max_pressed_raw() && stepCount < RELEASE_STEPS)
    {
        wait_if_switch_off();
        y_step_raw();
        stepCount++;
    }

    return stepCount;
}


/* ============================================================
   HOMING / LİMİT ARALIĞI ÖĞRENME
   ============================================================ */

static void home_x_axis(void)
{
    int32_t stepCount = 0;
    uint32_t releaseSteps = 0U;

    x_set_direction(DIR_MIN);

    while (!debounce_limit(x_min_pressed_raw))
    {
        wait_if_switch_off();
        x_step_raw();
    }

    delay_ms_with_alarm(100U);

    release_x_min();
    xPosition = 0;

    x_set_direction(DIR_MAX);
    stepCount = 0;

    while (!debounce_limit(x_max_pressed_raw))
    {
        wait_if_switch_off();
        x_step_raw();
        stepCount++;
    }

    xMaxPosition = stepCount;
    xPosition = xMaxPosition;

    delay_ms_with_alarm(100U);

    releaseSteps = release_x_max();

    if (xMaxPosition > (int32_t)releaseSteps)
    {
        xPosition = xMaxPosition - (int32_t)releaseSteps;
    }
    else
    {
        xPosition = 0;
    }
}

static void home_y_axis(void)
{
    int32_t stepCount = 0;
    uint32_t releaseSteps = 0U;

    y_set_direction(DIR_MIN);

    while (!debounce_limit(y_min_pressed_raw))
    {
        wait_if_switch_off();
        y_step_raw();
    }

    delay_ms_with_alarm(100U);

    release_y_min();
    yPosition = 0;

    y_set_direction(DIR_MAX);
    stepCount = 0;

    while (!debounce_limit(y_max_pressed_raw))
    {
        wait_if_switch_off();
        y_step_raw();
        stepCount++;
    }

    yMaxPosition = stepCount;
    yPosition = yMaxPosition;

    delay_ms_with_alarm(100U);

    releaseSteps = release_y_max();

    if (yMaxPosition > (int32_t)releaseSteps)
    {
        yPosition = yMaxPosition - (int32_t)releaseSteps;
    }
    else
    {
        yPosition = 0;
    }
}


/* ============================================================
   RANDOM HEDEF ÜRETME
   ============================================================ */

static uint32_t random_u32(void)
{
    randomSeed = randomSeed * 1664525U + 1013904223U;
    return randomSeed;
}

static int32_t random_target_between_limits(int32_t maxPosition)
{
    int32_t minTarget;
    int32_t maxTarget;
    int32_t range;

    if (maxPosition <= 0)
    {
        return 0;
    }

    minTarget = SAFETY_MARGIN_STEPS;
    maxTarget = maxPosition - SAFETY_MARGIN_STEPS;

    if (maxTarget <= minTarget)
    {
        minTarget = 0;
        maxTarget = maxPosition;
    }

    range = maxTarget - minTarget;

    if (range <= 0)
    {
        return 0;
    }

    return minTarget + (int32_t)(random_u32() % (uint32_t)(range + 1));
}


/* ============================================================
   EŞ ZAMANLI HAREKET
   ============================================================ */

static int32_t abs_i32(int32_t value)
{
    if (value < 0)
    {
        return -value;
    }

    return value;
}

static uint32_t get_move_delay_us(uint8_t stepX, uint8_t stepY)
{
    uint32_t delayValue = X_STEP_DELAY_US;
    uint32_t yDelay;

    if (stepY)
    {
        yDelay = get_y_step_delay_us();

        if (yDelay > delayValue)
        {
            delayValue = yDelay;
        }
    }

    if (!stepX && stepY)
    {
        delayValue = get_y_step_delay_us();
    }

    return delayValue;
}

static void pulse_axes(uint8_t stepX, uint8_t stepY)
{
    uint32_t delayValue;

    alarm_service();

    delayValue = get_move_delay_us(stepX, stepY);

    if (stepX)
    {
        gpio_set(GPIOB, X_STEP_PIN);
    }

    if (stepY)
    {
        gpio_set(GPIOB, Y_STEP_PIN);
    }

    delay_us(STEP_PULSE_HIGH_US);

    if (stepX)
    {
        gpio_reset(GPIOB, X_STEP_PIN);
    }

    if (stepY)
    {
        gpio_reset(GPIOB, Y_STEP_PIN);
    }

    if (stepX)
    {
        if (xDirection == DIR_MAX)
        {
            xPosition++;
        }
        else
        {
            xPosition--;
        }
    }

    if (stepY)
    {
        if (yDirection == DIR_MAX)
        {
            yPosition++;
        }
        else
        {
            yPosition--;
        }
    }

    delay_us(delayValue);
}

static void move_to_target(int32_t targetX, int32_t targetY)
{
    int32_t dx;
    int32_t dy;

    int32_t absX;
    int32_t absY;

    int32_t totalSteps;

    int32_t xError = 0;
    int32_t yError = 0;

    int32_t i;

    uint8_t stepX;
    uint8_t stepY;

    if (targetX < 0)
    {
        targetX = 0;
    }

    if (targetY < 0)
    {
        targetY = 0;
    }

    if (targetX > xMaxPosition)
    {
        targetX = xMaxPosition;
    }

    if (targetY > yMaxPosition)
    {
        targetY = yMaxPosition;
    }

    dx = targetX - xPosition;
    dy = targetY - yPosition;

    absX = abs_i32(dx);
    absY = abs_i32(dy);

    if ((absX == 0) && (absY == 0))
    {
        return;
    }

    if (dx > 0)
    {
        x_set_direction(DIR_MAX);
    }
    else if (dx < 0)
    {
        x_set_direction(DIR_MIN);
    }

    if (dy > 0)
    {
        y_set_direction(DIR_MAX);
    }
    else if (dy < 0)
    {
        y_set_direction(DIR_MIN);
    }

    if (absX > absY)
    {
        totalSteps = absX;
    }
    else
    {
        totalSteps = absY;
    }

    for (i = 0; i < totalSteps; i++)
    {
        wait_if_switch_off();

        if ((xDirection == DIR_MAX) && x_max_pressed_raw())
        {
            xPosition = xMaxPosition;
            release_x_max();
            break;
        }

        if ((xDirection == DIR_MIN) && x_min_pressed_raw())
        {
            xPosition = 0;
            release_x_min();
            break;
        }

        if ((yDirection == DIR_MAX) && y_max_pressed_raw())
        {
            yPosition = yMaxPosition;
            release_y_max();
            break;
        }

        if ((yDirection == DIR_MIN) && y_min_pressed_raw())
        {
            yPosition = 0;
            release_y_min();
            break;
        }

        stepX = 0U;
        stepY = 0U;

        if (absX > 0)
        {
            xError += absX;

            if (xError >= totalSteps)
            {
                xError -= totalSteps;

                if (xPosition != targetX)
                {
                    stepX = 1U;
                }
            }
        }

        if (absY > 0)
        {
            yError += absY;

            if (yError >= totalSteps)
            {
                yError -= totalSteps;

                if (yPosition != targetY)
                {
                    stepY = 1U;
                }
            }
        }

        pulse_axes(stepX, stepY);
    }
}


/* ============================================================
   MAIN
   ============================================================ */

int main(void)
{
    int32_t targetX;
    int32_t targetY;

    SystemCoreClockUpdate();

    dwt_init();

    gpio_init_all();

    adc1_init();

    /*
        LDR eşikleri burada belirlenir.
        Bu sırada lazer/ışık LDR'lere normal şekilde geliyor olmalı.
    */
    ldr_calibrate_thresholds();

    /*
        Motor anahtarı ON olana kadar bekle.
        Bu sırada alarm sistemi çalışmaya devam eder.
    */
    wait_if_switch_off();

    randomSeed ^= DWT->CYCCNT;

    /*
        Önce limit aralıklarını öğren.
        Homing sırasında önce X, sonra Y çalışır.
    */
    home_x_axis();
    home_y_axis();

    /*
        Sonsuz çalışma:
        Rastgele hedef seçilir.
        İki eksen birlikte hedefe gider.
        Alarm sürekli kontrol edilir.
    */
    while (1)
    {
        wait_if_switch_off();

        alarm_service();

        targetX = random_target_between_limits(xMaxPosition);
        targetY = random_target_between_limits(yMaxPosition);

        move_to_target(targetX, targetY);

        delay_ms_with_alarm(TARGET_WAIT_MS);
    }
}
