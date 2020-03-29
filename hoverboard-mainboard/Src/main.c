/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2017-2018 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2017-2018 Nico Stute <crinq@crinq.de>
* Copyright (C) 2017-2018 Niklas Fauth <niklas.fauth@kit.fail>
* Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdint.h>
#include <stdlib.h> // for abs()
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "config.h"

#include "BLDC_controller.h"      /* Model's header file */
#include "rtwtypes.h"

#include "../../common/Inc/SerialCommunication.h"

/*** Forward declarations ***/

void MX_GPIO_Init(void);
void MX_TIM_Init(void);
void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
void UART2_Init(void);
void UART3_Init(void);
void SystemClock_Config(void);
void poweroff(void);

/*** TIM, ADC, UART and DMA handles **/

TIM_HandleTypeDef htim_left;
TIM_HandleTypeDef htim_right;
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
//I2C_HandleTypeDef hi2c2;
//UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

//DMA_HandleTypeDef hdma_usart2_rx;
//DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;


/*** ADC buffer ***/

volatile adc_buf_t adc_buffer;

/*** left and right motor global struct ***/

struct
{
    RT_MODEL rtM; /* Real-time model */
    P rtP; /* Block parameters (auto storage) */
    DW rtDW; /* Observable states */
    ExtU rtU; /* External inputs */
    ExtY rtY; /* External outputs */

    //TODO initialize with default settings
    MotorSettings settings; /* Settings of Motor */

    MotorInput input; /* Data for Motor */

    uint8_t errCode; /* Global variable to handle Motor error codes */

} leftMotor, rightMotor;

/*** Default values for rtP (used for Init) ***/
extern const P rtP_Left;


/*** timeout variables ***/

volatile uint32_t timeout; // global variable for timeout

static int16_t timeoutCntADC   = 0;  // Timeout counter for ADC Protection
static uint8_t timeoutFlagADC  = 0;  // Timeout Flag for for ADC Protection: 0 = OK, 1 = Problem detected (line disconnected or wrong ADC data)

static int16_t timeoutCntSerial   = 0;  // Timeout counter for Rx Serial command
static uint8_t timeoutFlagSerial  = 0;  // Timeout Flag for Rx Serial command: 0 = OK, 1 = Problem detected (line disconnected or wrong Rx data)

static uint32_t inactivity_timeout_counter;

/*** Battery voltage variable ***/

int16_t        batVoltage       = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE;
static int32_t batVoltageFixdt  = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE << 20;  // Fixed-point filter output initialized at 400 V*100/cell = 4 V/cell converted to fixed-point

/*** Buzzer ***/
uint8_t buzzerFreq = 0;              // global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
uint8_t buzzerPattern = 0;           // global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...
static uint32_t buzzerTimer = 0;



static const uint16_t pwm_res  = 64000000 / 2 / PWM_FREQ; // = 2000
static const int16_t pwm_margin = 100;

static uint16_t offsetcount = 0;
static int16_t offsetrlA    = 2000;
static int16_t offsetrlB    = 2000;
static int16_t offsetrrB    = 2000;
static int16_t offsetrrC    = 2000;
static int16_t offsetdcl    = 2000;
static int16_t offsetdcr    = 2000;

uint32_t main_loop_counter;

/*** Serial communication ***/
SerialFeedback feedback;
SerialCommand command;

void receiveCommand(void);

void BLDC_Init(void) {
  /* Set default settings and default input */
  leftMotor.settings = defaultMotorSettings();
  rightMotor.settings = defaultMotorSettings();

  leftMotor.input.pwm = 0;
  rightMotor.input.pwm = 0;

  /* Set BLDC controller parameters */

  /* LEFT motor settings */
  leftMotor.rtP = rtP_Left;
  leftMotor.rtP.b_selPhaABCurrMeas   = 1;            // Left motor measured current phases {Green, Blue} = {iA, iB} -> do NOT change
  leftMotor.rtP.z_ctrlTypSel         = (uint8_t)leftMotor.settings.controlType;
  leftMotor.rtP.b_diagEna            = leftMotor.settings.diagnosticsEnable;
  leftMotor.rtP.i_max                = (leftMotor.settings.iMotorMax * A2BIT_CONV) << 4;        // fixdt(1,16,4)
  leftMotor.rtP.n_max                = leftMotor.settings.nMotorMax << 4;                       // fixdt(1,16,4)
  leftMotor.rtP.b_fieldWeakEna       = leftMotor.settings.fieldWeakeningEnable;
  leftMotor.rtP.id_fieldWeakMax      = (leftMotor.settings.fieldWeakeningMax * A2BIT_CONV) << 4;   // fixdt(1,16,4)
  leftMotor.rtP.a_phaAdvMax          = leftMotor.settings.fieldWeakeningPhaseAdvanceMax << 4;                   // fixdt(1,16,4)
  leftMotor.rtP.r_fieldWeakHi        = leftMotor.settings.fieldWeakeningHigh << 4;                   // fixdt(1,16,4)
  leftMotor.rtP.r_fieldWeakLo        = leftMotor.settings.fieldWeakeningLow << 4;                   // fixdt(1,16,4)

  /* Pack LEFT motor data into RTM */
  leftMotor.rtM.defaultParam        = &leftMotor.rtP;
  leftMotor.rtM.dwork               = &leftMotor.rtDW;
  leftMotor.rtM.inputs              = &leftMotor.rtU;
  leftMotor.rtM.outputs             = &leftMotor.rtY;

  /* RIGHT motor settings */
  rightMotor.rtP = rtP_Left;
  rightMotor.rtP.b_selPhaABCurrMeas  = 0;            // Right motor measured current phases {Blue, Yellow} = {iB, iC} -> do NOT change
  rightMotor.rtP.z_ctrlTypSel         = (uint8_t)rightMotor.settings.controlType;
  rightMotor.rtP.b_diagEna            = rightMotor.settings.diagnosticsEnable;
  rightMotor.rtP.i_max                = (rightMotor.settings.iMotorMax * A2BIT_CONV) << 4;        // fixdt(1,16,4)
  rightMotor.rtP.n_max                = rightMotor.settings.nMotorMax << 4;                       // fixdt(1,16,4)
  rightMotor.rtP.b_fieldWeakEna       = rightMotor.settings.fieldWeakeningEnable;
  rightMotor.rtP.id_fieldWeakMax      = (rightMotor.settings.fieldWeakeningMax * A2BIT_CONV) << 4;   // fixdt(1,16,4)
  rightMotor.rtP.a_phaAdvMax          = rightMotor.settings.fieldWeakeningPhaseAdvanceMax << 4;                   // fixdt(1,16,4)
  rightMotor.rtP.r_fieldWeakHi        = rightMotor.settings.fieldWeakeningHigh << 4;                   // fixdt(1,16,4)
  rightMotor.rtP.r_fieldWeakLo        = rightMotor.settings.fieldWeakeningLow << 4;                   // fixdt(1,16,4)

  /* Pack RIGHT motor data into RTM */
  rightMotor.rtM.defaultParam        = &rightMotor.rtP;
  rightMotor.rtM.dwork               = &rightMotor.rtDW;
  rightMotor.rtM.inputs              = &rightMotor.rtU;
  rightMotor.rtM.outputs             = &rightMotor.rtY;

  /* Initialize BLDC controllers */
  BLDC_controller_initialize(&leftMotor.rtM);
  BLDC_controller_initialize(&rightMotor.rtM);
}

void poweronMelody(void)
{
    for (int i = 8; i >= 0; i--) {
        buzzerFreq = (uint8_t)i;
        HAL_Delay(100);
    }

    buzzerFreq = 0;
}

void shortBeep(uint8_t freq)
{
    buzzerFreq = freq;
    HAL_Delay(100);
    buzzerFreq = 0;
}

void shortBeepMany(uint8_t cnt)
{
    for(uint8_t i = 0; i < cnt; i++)
    {
      shortBeep(i + 5);
    }
}

void longBeep(uint8_t freq)
{
    buzzerFreq = freq;
    HAL_Delay(500);
    buzzerFreq = 0;
}

/* =========================== Filtering Functions =========================== */

  /* Low pass filter fixed-point 32 bits: fixdt(1,32,20)
  * Max:  2047.9375
  * Min: -2048
  * Res:  0.0625
  *
  * Inputs:       u     = int16 or int32
  * Outputs:      y     = fixdt(1,32,16)
  * Parameters:   coef  = fixdt(0,16,16) = [0,65535U]
  *
  * Example:
  * If coef = 0.8 (in floating point), then coef = 0.8 * 2^16 = 52429 (in fixed-point)
  * filtLowPass16(u, 52429, &y);
  * yint = (int16_t)(y >> 16); // the integer output is the fixed-point ouput shifted by 16 bits
  */
 void filtLowPass32(int32_t u, uint16_t coef, int32_t *y) {
  int64_t tmp;
  tmp = ((int64_t)((u << 4) - (*y >> 12)) * coef) >> 4;
  tmp = CLAMP(tmp, -2147483648LL, 2147483647LL);  // Overflow protection: 2147483647LL = 2^31 - 1
  *y = (int32_t)tmp + (*y);
}

int main(void)
{
    HAL_Init();
    __HAL_RCC_AFIO_CLK_ENABLE();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    /* System interrupt init*/
    /* MemoryManagement_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
    /* BusFault_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
    /* UsageFault_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
    /* SVCall_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
    /* DebugMonitor_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
    /* PendSV_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

    SystemClock_Config();

    __HAL_RCC_DMA1_CLK_DISABLE();
    MX_GPIO_Init();
    MX_TIM_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();

    BLDC_Init();        // BLDC Controller Init

    HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_SET);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_Start(&hadc2);


    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);


    poweronMelody();

    UART3_Init();

    int16_t speed = 0;

    int16_t speedL     = 0, speedR     = 0;
    int16_t lastSpeedL = 0, lastSpeedR = 0;

    int16_t    speedAvg;             // average measured speed
    int16_t    speedAvgAbs;          // average measured speed in absolute

    /* Board temperature */

    int32_t board_temp_adcFixdt = adc_buffer.temp << 20;  // Fixed-point filter output initialized with current ADC converted to fixed-point
    int16_t board_temp_adcFilt  = adc_buffer.temp;
    int16_t board_temp_deg_c;

    HAL_UART_Receive_DMA(&huart3, (uint8_t *)&command, sizeof(command));

    while(1)
    {
        HAL_Delay(DELAY_IN_MAIN_LOOP); //delay in ms

        // ####### Get serial Commands from microcontroller #######
        receiveCommand();

        // ### ADC control of motors ###

        // Get the ADC inputs from potis
        double leftPoti = CLAMP((adc_buffer.l_tx2 - ADC1_MIN) * 1000. / (ADC1_MAX - ADC1_MIN), 0., 1000.);    // ADC1
        double rightPoti = CLAMP((adc_buffer.l_rx2 - ADC2_MIN) * 1000. / (ADC2_MAX - ADC2_MIN), 0., 1000.);    // ADC2

        // Smooth steping the input
        double a = 0.;
        double b = 1000.;

        double t1 = MIN(
                    MAX(
                        (leftPoti-a)/(b-a),
                        0
                        ),
                    1
                    );
        leftPoti = t1*t1*t1*1000.;

        double t2 = MIN(
                    MAX(
                        (rightPoti-a)/(b-a),
                        0
                        ),
                    1
                    );
        rightPoti = t2*t2*t2*1000.;

        timeout = 0;

        if (leftPoti >= 950.)
            speed = leftPoti + (rightPoti / 2.);
        else
            speed = leftPoti - (rightPoti * 0.75);

        speedL = speedR = speed;

        /// ####### SET OUTPUTS (if the target change is less than +/- 50) #######
        if ((speedL > lastSpeedL-50 && speedL < lastSpeedL+50) && (speedR > lastSpeedR-50 && speedR < lastSpeedR+50) && timeout < TIMEOUT)
        {
            // use calculated ADC input
            if(!command.overrideMotorInput)
            {
                leftMotor.input.pwm = speedL;

                rightMotor.input.pwm = -speedR;
            }

            // else use serial input for pwm from microcontroller
            else
            {
                lastSpeedL = command.leftMotorInput.pwm;
                lastSpeedR = command.rightMotorInput.pwm;

                //leftMotor.input = command.leftMotorInput;
                //rightMotor.input = command.rightMotorInput;
            }

        }

        if(!command.overrideMotorInput)
        {
            lastSpeedL = speedL;
            lastSpeedR = speedR;
        }

        timeout = 0;

        // Calculate average measured speed: speedAvg, speedAvgAbs
        // Calculate measured average speed. The minus sign (-) is because motors spin in opposite directions
        speedAvg    = ( leftMotor.rtY.n_mot - rightMotor.rtY.n_mot) / 2;
        // Handle the case when SPEED_COEFFICIENT sign is negative (which is when most significant bit is 1)
        if (SPEED_COEFFICIENT & (1 << 16)) {
          speedAvg    = -speedAvg;
        }
        speedAvgAbs   = abs(speedAvg);

        // ####### CALC BOARD TEMPERATURE #######
        filtLowPass32(adc_buffer.temp, TEMP_FILT_COEF, &board_temp_adcFixdt);
        board_temp_adcFilt  = (int16_t)(board_temp_adcFixdt >> 16);  // convert fixed-point to integer
        board_temp_deg_c    = (TEMP_CAL_HIGH_DEG_C - TEMP_CAL_LOW_DEG_C) * (board_temp_adcFilt - TEMP_CAL_LOW_ADC) / (TEMP_CAL_HIGH_ADC - TEMP_CAL_LOW_ADC) + TEMP_CAL_LOW_DEG_C;

        // ####### FEEDBACK SERIAL OUT #######
        if (main_loop_counter % 2 == 0)
        {
            // Send data periodically every 10 ms
            feedback.start	        = (uint16_t)SERIAL_START_FRAME;
            feedback.cmd1           = (int16_t)leftPoti;
            feedback.cmd2           = (int16_t)rightPoti;
            feedback.adc1           = (int16_t)adc_buffer.l_tx2;
            feedback.adc2           = (int16_t)adc_buffer.l_rx2;
            feedback.pwm1           = (int16_t)leftMotor.input.pwm;
            feedback.pwm2           = (int16_t)rightMotor.input.pwm;
            feedback.speedR_meas	  = (int16_t)rightMotor.rtY.n_mot;
            feedback.speedL_meas	  = (int16_t)leftMotor.rtY.n_mot;
            feedback.batVoltage	    = (int16_t)(batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC);
            feedback.boardTemp	    = (int16_t)board_temp_deg_c;

            feedback.leftMotorSettings = leftMotor.settings;
            feedback.rightMotorSettings = rightMotor.settings;

            feedback.leftMotorInput = leftMotor.input;
            feedback.rightMotorInput = rightMotor.input;

            if(DMA1_Channel2->CNDTR == 0)
            {
                feedback.cmdLed         = (uint16_t)0;
                feedback.checksum       = (uint16_t)calculateChecksumFeedback(feedback);
                DMA1_Channel2->CCR     &= ~DMA_CCR_EN;
                DMA1_Channel2->CNDTR    = sizeof(feedback);
                DMA1_Channel2->CMAR     = (uint32_t)&feedback;
                DMA1_Channel2->CCR     |= DMA_CCR_EN;
            }
        }

        // ####### POWEROFF BY POWER-BUTTON #######
        if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN))
        {
            leftMotor.settings.enable = false;
            rightMotor.settings.enable = false;

            // disable motors
            while (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {}    // wait until button is released
            if(__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {               // do not power off after software reset (from a programmer/debugger)
                __HAL_RCC_CLEAR_RESET_FLAGS();                        // clear reset flags
            } else {
                poweroff();                                           // release power-latch
            }
        }

        // ####### BEEP AND EMERGENCY POWEROFF #######
        if ((TEMP_POWEROFF_ENABLE && board_temp_deg_c >= TEMP_POWEROFF && speedAvgAbs < 20) || (batVoltage < BAT_DEAD && speedAvgAbs < 20)) {  // poweroff before mainboard burns OR low bat 3
          poweroff();
        } else if (leftMotor.rtY.z_errCode || rightMotor.rtY.z_errCode) {     // disable motors and beep in case of Motor error - fast beep
          leftMotor.settings.enable = false;
          rightMotor.settings.enable = false;
          buzzerFreq    = 8;
          buzzerPattern = 1;
        } else if (timeoutFlagADC || timeoutFlagSerial) {           // beep in case of ADC or Serial timeout - fast beep
          buzzerFreq    = 20;
          buzzerPattern = 1;
        } else if (TEMP_WARNING_ENABLE && board_temp_deg_c >= TEMP_WARNING) {  // beep if mainboard gets hot
          buzzerFreq    = 4;
          buzzerPattern = 1;
        } else if (BAT_LVL1_ENABLE && batVoltage < BAT_LVL1) {      // low bat 1: fast beep
          buzzerFreq    = 5;
          buzzerPattern = 6;
        } else if (BAT_LVL2_ENABLE && batVoltage < BAT_LVL2) {      // low bat 2: slow beep
          buzzerFreq    = 5;
          buzzerPattern = 42;
        } else if (BEEPS_BACKWARD && speed < -50 && speedAvg < 0) {  // backward beep
          buzzerFreq    = 5;
          buzzerPattern = 1;
        } else {  // do not beep
          buzzerFreq    = 0;
          buzzerPattern = 0;
        }


        // ####### INACTIVITY TIMEOUT #######
        if (abs(speedL) > 50 || abs(speedR) > 50) {
          inactivity_timeout_counter = 0;
        } else {
          inactivity_timeout_counter ++;
        }
        if (inactivity_timeout_counter > (INACTIVITY_TIMEOUT * 60 * 1000) / (DELAY_IN_MAIN_LOOP + 1)) {  // rest of main loop needs maybe 1ms
          poweroff();
        }

        //HAL_GPIO_TogglePin(LED_PORT, LED_PIN);                 // This is to measure the main() loop duration with an oscilloscope connected to LED_PIN

        // Update main loop states
        main_loop_counter++;
        timeout++;
    }
}

int16_t curL_phaA = 0, curL_phaB = 0, curL_DC = 0;
int16_t curR_phaB = 0, curR_phaC = 0, curR_DC = 0;

void DMA1_Channel1_IRQHandler(void)
{

  DMA1->IFCR = DMA_IFCR_CTCIF1;
  // HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);
  //HAL_GPIO_TogglePin(LED_PORT, LED_PIN);

  if(offsetcount < 2000) {  // calibrate ADC offsets
    offsetcount++;
    offsetrlA = (adc_buffer.rlA + offsetrlA) / 2;
    offsetrlB = (adc_buffer.rlB + offsetrlB) / 2;
    offsetrrB = (adc_buffer.rrB + offsetrrB) / 2;
    offsetrrC = (adc_buffer.rrC + offsetrrC) / 2;
    offsetdcl = (adc_buffer.dcl + offsetdcl) / 2;
    offsetdcr = (adc_buffer.dcr + offsetdcr) / 2;
    return;
  }

  if (buzzerTimer % 1000 == 0) {  // because you get float rounding errors if it would run every time -> not any more, everything converted to fixed-point
    filtLowPass32(adc_buffer.batt1, BAT_FILT_COEF, &batVoltageFixdt);
    batVoltage = (int16_t)(batVoltageFixdt >> 16);  // convert fixed-point to integer
  }

  // Get Left motor currents
  curL_phaA = (int16_t)(offsetrlA - adc_buffer.rlA);
  curL_phaB = (int16_t)(offsetrlB - adc_buffer.rlB);
  curL_DC   = (int16_t)(offsetdcl - adc_buffer.dcl);

  // Get Right motor currents
  curR_phaB = (int16_t)(offsetrrB - adc_buffer.rrB);
  curR_phaC = (int16_t)(offsetrrC - adc_buffer.rrC);
  curR_DC   = (int16_t)(offsetdcr - adc_buffer.dcr);

  // Disable PWM when current limit is reached (current chopping)
  // This is the Level 2 of current protection. The Level 1 should kick in first given by I_MOT_MAX
  if(ABS(curL_DC) > (leftMotor.settings.iDcCurrentMax * A2BIT_CONV) || timeout > TIMEOUT || leftMotor.settings.enable == false) {
    LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
  } else {
    LEFT_TIM->BDTR |= TIM_BDTR_MOE;
  }

  if(ABS(curR_DC)  > (rightMotor.settings.iDcCurrentMax * A2BIT_CONV) || timeout > TIMEOUT || leftMotor.settings.enable == false) {
    RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
  } else {
    RIGHT_TIM->BDTR |= TIM_BDTR_MOE;
  }

  //create square wave for buzzer
  buzzerTimer++;

  if (buzzerFreq != 0 && (buzzerTimer / 5000) % (buzzerPattern + 1) == 0) {
    if (buzzerTimer % buzzerFreq == 0) {
      HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
    }
  } else {
      HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
  }

  // ############################### MOTOR CONTROL ###############################

  int ul, vl, wl;
  int ur, vr, wr;
  static boolean_T OverrunFlag = false;

  /* Check for overrun */
  if (OverrunFlag) {
    return;
  }
  OverrunFlag = true;

  /* Make sure to stop BOTH motors in case of an error */
  bool leftMotorEnableFin = leftMotor.settings.enable  && !leftMotor.rtY.z_errCode && !rightMotor.rtY.z_errCode;
  bool rightMotorEnableFin = rightMotor.settings.enable  && !leftMotor.rtY.z_errCode && !rightMotor.rtY.z_errCode;

  // ========================= LEFT MOTOR ============================
  // Get hall sensors values
  uint8_t hall_ul = !(LEFT_HALL_U_PORT->IDR & LEFT_HALL_U_PIN);
  uint8_t hall_vl = !(LEFT_HALL_V_PORT->IDR & LEFT_HALL_V_PIN);
  uint8_t hall_wl = !(LEFT_HALL_W_PORT->IDR & LEFT_HALL_W_PIN);

  /* Update LEFT Motor Settings */
  leftMotor.rtP.b_selPhaABCurrMeas   = 1;            // Left motor measured current phases {Green, Blue} = {iA, iB} -> do NOT change
  leftMotor.rtP.z_ctrlTypSel         = (uint8_t)leftMotor.settings.controlType; //TODO uint8 ?
  leftMotor.rtP.b_diagEna            = leftMotor.settings.diagnosticsEnable;
  leftMotor.rtP.i_max                = (leftMotor.settings.iMotorMax * A2BIT_CONV) << 4;        // fixdt(1,16,4)
  leftMotor.rtP.n_max                = leftMotor.settings.nMotorMax << 4;                       // fixdt(1,16,4)
  leftMotor.rtP.b_fieldWeakEna       = leftMotor.settings.fieldWeakeningEnable;
  leftMotor.rtP.id_fieldWeakMax      = (leftMotor.settings.fieldWeakeningMax * A2BIT_CONV) << 4;   // fixdt(1,16,4)
  leftMotor.rtP.a_phaAdvMax          = leftMotor.settings.fieldWeakeningPhaseAdvanceMax << 4;                   // fixdt(1,16,4)
  leftMotor.rtP.r_fieldWeakHi        = leftMotor.settings.fieldWeakeningHigh << 4;                   // fixdt(1,16,4)
  leftMotor.rtP.r_fieldWeakLo        = leftMotor.settings.fieldWeakeningLow << 4;                   // fixdt(1,16,4)

  /* Set LEFT motor inputs here */
  leftMotor.rtU.b_motEna     = leftMotorEnableFin;
  leftMotor.rtU.z_ctrlModReq = (uint8_t)leftMotor.settings.controlMode;
  leftMotor.rtU.r_inpTgt     = leftMotor.input.pwm;
  leftMotor.rtU.b_hallA      = hall_ul;
  leftMotor.rtU.b_hallB      = hall_vl;
  leftMotor.rtU.b_hallC      = hall_wl;
  leftMotor.rtU.i_phaAB      = curL_phaA;
  leftMotor.rtU.i_phaBC      = curL_phaB;
  leftMotor.rtU.i_DCLink     = curL_DC;

  /* Step the controller */
  BLDC_controller_step(&leftMotor.rtM);

  /* Get motor outputs here */
  ul            = leftMotor.rtY.DC_phaA;
  vl            = leftMotor.rtY.DC_phaB;
  wl            = leftMotor.rtY.DC_phaC;
  // errCodeLeft  = rtY_Left.z_errCode;
  // motSpeedLeft = rtY_Left.n_mot;
  // motAngleLeft = rtY_Left.a_elecAngle;

  /* Apply commands */
  LEFT_TIM->LEFT_TIM_U    = (uint16_t)CLAMP(ul + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
  LEFT_TIM->LEFT_TIM_V    = (uint16_t)CLAMP(vl + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
  LEFT_TIM->LEFT_TIM_W    = (uint16_t)CLAMP(wl + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
  // =================================================================


  // ========================= RIGHT MOTOR ===========================
  // Get hall sensors values
  uint8_t hall_ur = !(RIGHT_HALL_U_PORT->IDR & RIGHT_HALL_U_PIN);
  uint8_t hall_vr = !(RIGHT_HALL_V_PORT->IDR & RIGHT_HALL_V_PIN);
  uint8_t hall_wr = !(RIGHT_HALL_W_PORT->IDR & RIGHT_HALL_W_PIN);


  /* Update RIGHT motor settings */
  rightMotor.rtP.b_selPhaABCurrMeas  = 0;            // Right motor measured current phases {Blue, Yellow} = {iB, iC} -> do NOT change
  rightMotor.rtP.z_ctrlTypSel         = (uint8_t)rightMotor.settings.controlType;
  rightMotor.rtP.b_diagEna            = rightMotor.settings.diagnosticsEnable;
  rightMotor.rtP.i_max                = (rightMotor.settings.iMotorMax * A2BIT_CONV) << 4;        // fixdt(1,16,4)
  rightMotor.rtP.n_max                = rightMotor.settings.nMotorMax << 4;                       // fixdt(1,16,4)
  rightMotor.rtP.b_fieldWeakEna       = rightMotor.settings.fieldWeakeningEnable;
  rightMotor.rtP.id_fieldWeakMax      = (rightMotor.settings.fieldWeakeningMax * A2BIT_CONV) << 4;   // fixdt(1,16,4)
  rightMotor.rtP.a_phaAdvMax          = rightMotor.settings.fieldWeakeningPhaseAdvanceMax << 4;                   // fixdt(1,16,4)
  rightMotor.rtP.r_fieldWeakHi        = rightMotor.settings.fieldWeakeningHigh << 4;                   // fixdt(1,16,4)
  rightMotor.rtP.r_fieldWeakLo        = rightMotor.settings.fieldWeakeningLow << 4;                   // fixdt(1,16,4)

  /* Set RIGHT motor inputs here */
  rightMotor.rtU.b_motEna      = rightMotorEnableFin;
  rightMotor.rtU.z_ctrlModReq  = (uint8_t)rightMotor.settings.controlMode;
  rightMotor.rtU.r_inpTgt      = rightMotor.input.pwm;
  rightMotor.rtU.b_hallA       = hall_ur;
  rightMotor.rtU.b_hallB       = hall_vr;
  rightMotor.rtU.b_hallC       = hall_wr;
  rightMotor.rtU.i_phaAB       = curR_phaB;
  rightMotor.rtU.i_phaBC       = curR_phaC;
  rightMotor.rtU.i_DCLink      = curR_DC;

  /* Step the controller */
  BLDC_controller_step(&rightMotor.rtM);

  /* Get motor outputs here */
  ur            = rightMotor.rtY.DC_phaA;
  vr            = rightMotor.rtY.DC_phaB;
  wr            = rightMotor.rtY.DC_phaC;
  // errCodeRight  = rtY_Right.z_errCode;
  // motSpeedRight = rtY_Right.n_mot;
  // motAngleRight = rtY_Right.a_elecAngle;

  /* Apply commands */
  RIGHT_TIM->RIGHT_TIM_U  = (uint16_t)CLAMP(ur + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
  RIGHT_TIM->RIGHT_TIM_V  = (uint16_t)CLAMP(vr + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
  RIGHT_TIM->RIGHT_TIM_W  = (uint16_t)CLAMP(wr + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
  // =================================================================

  /* Indicate task complete */
  OverrunFlag = false;

  // ###############################################################################

}

/* =========================== Poweroff Functions =========================== */

void poweroff(void) {
    buzzerPattern = 0;
    leftMotor.settings.enable = false;
    rightMotor.settings.enable = false;
    //consoleLog("-- Motors disabled --\r\n");
    for (int i = 0; i < 8; i++) {
        buzzerFreq = (uint8_t)i;
        HAL_Delay(100);
    }
    HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_RESET);
    while(1) {}
}

// ===========================================================
/** System Clock Configuration
*/
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType           = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource        = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider       = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider      = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider      = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection    = RCC_PERIPHCLK_ADC;
  // PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV8;  // 8 MHz
  PeriphClkInit.AdcClockSelection       = RCC_ADCPCLK2_DIV4;  // 16 MHz
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

// ===========================================================
/** Setup Functions ADC, UART etc.
*/


#if defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2) || \
    defined(FEEDBACK_SERIAL_USAR2) || defined(DEBUG_SERIAL_USART2)
void UART2_Init(void) {

  /* The code below is commented out - otwerwise Serial Receive does not work */
  // #if defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART3)
  //   /* DMA1_Channel6_IRQn interrupt configuration */
  //   HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 6);
  //   HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  //   /* DMA1_Channel7_IRQn interrupt configuration */
  //   HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 7);
  //   HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  // #endif

  // Disable serial interrupt - it is not needed
  HAL_NVIC_DisableIRQ(DMA1_Channel6_IRQn);    // Rx Channel
  HAL_NVIC_DisableIRQ(DMA1_Channel7_IRQn);    // Tx Channel

  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART2_CLK_ENABLE();

  huart2.Instance           = USART2;
  huart2.Init.BaudRate      = USART2_BAUD;
  huart2.Init.WordLength    = USART2_WORDLENGTH;
  huart2.Init.StopBits      = UART_STOPBITS_1;
  huart2.Init.Parity        = UART_PARITY_NONE;
  huart2.Init.HwFlowCtl     = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling  = UART_OVERSAMPLING_16;
  #if defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2)
    huart2.Init.Mode        = UART_MODE_TX_RX;
  #elif defined(DEBUG_SERIAL_USART2)
    huart2.Init.Mode        = UART_MODE_TX;
  #endif
  HAL_UART_Init(&huart2);

  #if defined(FEEDBACK_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2)
    USART2->CR3 |= USART_CR3_DMAT;  // | USART_CR3_DMAR | USART_CR3_OVRDIS;
  #endif

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin       = GPIO_PIN_2;
  GPIO_InitStruct.Pull      = GPIO_PULLUP; //GPIO_NOPULL;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  #if defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2)
    GPIO_InitStruct.Pin     = GPIO_PIN_3;
    GPIO_InitStruct.Mode    = GPIO_MODE_INPUT; //GPIO_MODE_AF_PP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Peripheral DMA init*/
    hdma_usart2_rx.Instance                 = DMA1_Channel6;
    hdma_usart2_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode                = DMA_CIRCULAR; //DMA_NORMAL;
    hdma_usart2_rx.Init.Priority            = DMA_PRIORITY_LOW;
    HAL_DMA_Init(&hdma_usart2_rx);
    __HAL_LINKDMA(&huart2, hdmarx, hdma_usart2_rx);
  #endif

  hdma_usart2_tx.Instance                   = DMA1_Channel7;
  hdma_usart2_tx.Init.Direction             = DMA_MEMORY_TO_PERIPH;
  hdma_usart2_tx.Init.PeriphInc             = DMA_PINC_DISABLE;
  hdma_usart2_tx.Init.MemInc                = DMA_MINC_ENABLE;
  hdma_usart2_tx.Init.PeriphDataAlignment   = DMA_PDATAALIGN_BYTE;
  hdma_usart2_tx.Init.MemDataAlignment      = DMA_MDATAALIGN_BYTE;
  hdma_usart2_tx.Init.Mode                  = DMA_NORMAL;
  hdma_usart2_tx.Init.Priority              = DMA_PRIORITY_LOW;
  HAL_DMA_Init(&hdma_usart2_tx);

  #if defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2)
    __HAL_LINKDMA(&huart2, hdmatx, hdma_usart2_tx);
  #endif
  #if defined(FEEDBACK_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2)
    DMA1_Channel7->CPAR     = (uint32_t) & (USART2->DR);
    DMA1_Channel7->CNDTR    = 0;
    DMA1->IFCR              = DMA_IFCR_CTCIF7 | DMA_IFCR_CHTIF7 | DMA_IFCR_CGIF7;
  #endif

}
#endif

#if defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3) || \
    defined(FEEDBACK_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3)
void UART3_Init(void) {

  /* The code below is commented out - otwerwise Serial Receive does not work */
  // #if defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3)
  //   /* DMA1_Channel3_IRQn interrupt configuration */
  //   HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 3);
  //   HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  //   /* DMA1_Channel2_IRQn interrupt configuration */
  //   HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 2);
  //   HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  // #endif

  // Disable serial interrupt - it is not needed
  HAL_NVIC_DisableIRQ(DMA1_Channel3_IRQn);  // Rx Channel
  HAL_NVIC_DisableIRQ(DMA1_Channel2_IRQn);  // Tx Channel

  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_USART3_CLK_ENABLE();

  huart3.Instance             = USART3;
  huart3.Init.BaudRate        = USART3_BAUD;
  huart3.Init.WordLength      = USART3_WORDLENGTH;
  huart3.Init.StopBits        = UART_STOPBITS_1;
  huart3.Init.Parity          = UART_PARITY_NONE;
  huart3.Init.HwFlowCtl       = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling    = UART_OVERSAMPLING_16;
  #if defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3)
    huart3.Init.Mode          = UART_MODE_TX_RX;
  #elif defined(DEBUG_SERIAL_USART3)
    huart3.Init.Mode          = UART_MODE_TX;
  #endif
  HAL_UART_Init(&huart3);

  #if defined(FEEDBACK_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3)
    USART3->CR3 |= USART_CR3_DMAT;  // | USART_CR3_DMAR | USART_CR3_OVRDIS;
  #endif

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin         = GPIO_PIN_10;
  GPIO_InitStruct.Pull        = GPIO_PULLUP;
  GPIO_InitStruct.Mode        = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed       = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  #if defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3)
    GPIO_InitStruct.Pin       = GPIO_PIN_11;
    GPIO_InitStruct.Mode      = GPIO_MODE_INPUT;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral DMA init*/
    hdma_usart3_rx.Instance                   = DMA1_Channel3;
    hdma_usart3_rx.Init.Direction             = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc             = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc                = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment   = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment      = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode                  = DMA_CIRCULAR; //DMA_NORMAL;
    hdma_usart3_rx.Init.Priority              = DMA_PRIORITY_LOW;
    HAL_DMA_Init(&hdma_usart3_rx);
    __HAL_LINKDMA(&huart3, hdmarx, hdma_usart3_rx);
  #endif

  hdma_usart3_tx.Instance                     = DMA1_Channel2;
  hdma_usart3_tx.Init.Direction               = DMA_MEMORY_TO_PERIPH;
  hdma_usart3_tx.Init.PeriphInc               = DMA_PINC_DISABLE;
  hdma_usart3_tx.Init.MemInc                  = DMA_MINC_ENABLE;
  hdma_usart3_tx.Init.PeriphDataAlignment     = DMA_PDATAALIGN_BYTE;
  hdma_usart3_tx.Init.MemDataAlignment        = DMA_MDATAALIGN_BYTE;
  hdma_usart3_tx.Init.Mode                    = DMA_NORMAL;
  hdma_usart3_tx.Init.Priority                = DMA_PRIORITY_LOW;
  HAL_DMA_Init(&hdma_usart3_tx);

  #if defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3)
    __HAL_LINKDMA(&huart3, hdmatx, hdma_usart3_tx);
  #endif
  #if defined(FEEDBACK_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3)
    DMA1_Channel2->CPAR     = (uint32_t) & (USART3->DR);
    DMA1_Channel2->CNDTR    = 0;
    DMA1->IFCR              = DMA_IFCR_CTCIF2 | DMA_IFCR_CHTIF2 | DMA_IFCR_CGIF2;
  #endif
}
#endif

void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  GPIO_InitStruct.Pin = LEFT_HALL_U_PIN;
  HAL_GPIO_Init(LEFT_HALL_U_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_HALL_V_PIN;
  HAL_GPIO_Init(LEFT_HALL_V_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_HALL_W_PIN;
  HAL_GPIO_Init(LEFT_HALL_W_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_HALL_U_PIN;
  HAL_GPIO_Init(RIGHT_HALL_U_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_HALL_V_PIN;
  HAL_GPIO_Init(RIGHT_HALL_V_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_HALL_W_PIN;
  HAL_GPIO_Init(RIGHT_HALL_W_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = CHARGER_PIN;
  HAL_GPIO_Init(CHARGER_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = BUTTON_PIN;
  HAL_GPIO_Init(BUTTON_PORT, &GPIO_InitStruct);


  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

  GPIO_InitStruct.Pin = LED_PIN;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = BUZZER_PIN;
  HAL_GPIO_Init(BUZZER_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = OFF_PIN;
  HAL_GPIO_Init(OFF_PORT, &GPIO_InitStruct);


  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;

  GPIO_InitStruct.Pin = LEFT_DC_CUR_PIN;
  HAL_GPIO_Init(LEFT_DC_CUR_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_U_CUR_PIN;
  HAL_GPIO_Init(LEFT_U_CUR_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_V_CUR_PIN;
  HAL_GPIO_Init(LEFT_V_CUR_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_DC_CUR_PIN;
  HAL_GPIO_Init(RIGHT_DC_CUR_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_U_CUR_PIN;
  HAL_GPIO_Init(RIGHT_U_CUR_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_V_CUR_PIN;
  HAL_GPIO_Init(RIGHT_V_CUR_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = DCLINK_PIN;
  HAL_GPIO_Init(DCLINK_PORT, &GPIO_InitStruct);

  //Analog in
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;

  GPIO_InitStruct.Pin = LEFT_TIM_UH_PIN;
  HAL_GPIO_Init(LEFT_TIM_UH_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_TIM_VH_PIN;
  HAL_GPIO_Init(LEFT_TIM_VH_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_TIM_WH_PIN;
  HAL_GPIO_Init(LEFT_TIM_WH_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_TIM_UL_PIN;
  HAL_GPIO_Init(LEFT_TIM_UL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_TIM_VL_PIN;
  HAL_GPIO_Init(LEFT_TIM_VL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_TIM_WL_PIN;
  HAL_GPIO_Init(LEFT_TIM_WL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_TIM_UH_PIN;
  HAL_GPIO_Init(RIGHT_TIM_UH_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_TIM_VH_PIN;
  HAL_GPIO_Init(RIGHT_TIM_VH_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_TIM_WH_PIN;
  HAL_GPIO_Init(RIGHT_TIM_WH_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_TIM_UL_PIN;
  HAL_GPIO_Init(RIGHT_TIM_UL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_TIM_VL_PIN;
  HAL_GPIO_Init(RIGHT_TIM_VL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_TIM_WL_PIN;
  HAL_GPIO_Init(RIGHT_TIM_WL_PORT, &GPIO_InitStruct);
}

void MX_TIM_Init(void) {
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM8_CLK_ENABLE();

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_SlaveConfigTypeDef sTimConfig;

  htim_right.Instance               = RIGHT_TIM;
  htim_right.Init.Prescaler         = 0;
  htim_right.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
  htim_right.Init.Period            = 64000000 / 2 / PWM_FREQ;
  htim_right.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim_right.Init.RepetitionCounter = 0;
  htim_right.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_PWM_Init(&htim_right);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim_right, &sMasterConfig);

  sConfigOC.OCMode       = TIM_OCMODE_PWM1;
  sConfigOC.Pulse        = 0;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
  HAL_TIM_PWM_ConfigChannel(&htim_right, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim_right, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim_right, &sConfigOC, TIM_CHANNEL_3);

  sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime         = DEAD_TIME;
  sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim_right, &sBreakDeadTimeConfig);

  htim_left.Instance               = LEFT_TIM;
  htim_left.Init.Prescaler         = 0;
  htim_left.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
  htim_left.Init.Period            = 64000000 / 2 / PWM_FREQ;
  htim_left.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim_left.Init.RepetitionCounter = 0;
  htim_left.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_PWM_Init(&htim_left);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_ENABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim_left, &sMasterConfig);

  sTimConfig.InputTrigger = TIM_TS_ITR0;
  sTimConfig.SlaveMode    = TIM_SLAVEMODE_GATED;
  HAL_TIM_SlaveConfigSynchronization(&htim_left, &sTimConfig);

  // Start counting >0 to effectively offset timers by the time it takes for one ADC conversion to complete.
  // This method allows that the Phase currents ADC measurements are properly aligned with LOW-FET ON region for both motors
  LEFT_TIM->CNT 		     = ADC_TOTAL_CONV_TIME;

  sConfigOC.OCMode       = TIM_OCMODE_PWM1;
  sConfigOC.Pulse        = 0;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
  HAL_TIM_PWM_ConfigChannel(&htim_left, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim_left, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim_left, &sConfigOC, TIM_CHANNEL_3);

  sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime         = DEAD_TIME;
  sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim_left, &sBreakDeadTimeConfig);

  LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
  RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;

  HAL_TIM_PWM_Start(&htim_left, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim_left, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim_left, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim_left, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim_left, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim_left, TIM_CHANNEL_3);

  HAL_TIM_PWM_Start(&htim_right, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim_right, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim_right, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim_right, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim_right, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim_right, TIM_CHANNEL_3);

  htim_left.Instance->RCR = 1;

  __HAL_TIM_ENABLE(&htim_right);
}

void MX_ADC1_Init(void) {
  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

  __HAL_RCC_ADC1_CLK_ENABLE();

  hadc1.Instance                   = ADC1;
  hadc1.Init.ScanConvMode          = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode    = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion       = 5;
  HAL_ADC_Init(&hadc1);
  /**Enable or disable the remapping of ADC1_ETRGREG:
    * ADC1 External Event regular conversion is connected to TIM8 TRG0
    */
  __HAL_AFIO_REMAP_ADC1_ETRGREG_ENABLE();

  /**Configure the ADC multi-mode
    */
  multimode.Mode = ADC_DUALMODE_REGSIMULT;
  HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode);

  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.Channel = ADC_CHANNEL_11;  // pc1 left cur  ->  right
  sConfig.Rank    = 1;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  // sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  sConfig.Channel = ADC_CHANNEL_0;  // pa0 right a   ->  left
  sConfig.Rank    = 2;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_14;  // pc4 left b   -> right
  sConfig.Rank    = 3;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_12;  // pc2 vbat
  sConfig.Rank    = 4;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  //temperature requires at least 17.1uS sampling time
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;  // internal temp
  sConfig.Rank    = 5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  hadc1.Instance->CR2 |= ADC_CR2_DMA | ADC_CR2_TSVREFE;

  __HAL_ADC_ENABLE(&hadc1);

  __HAL_RCC_DMA1_CLK_ENABLE();

  DMA1_Channel1->CCR   = 0;
  DMA1_Channel1->CNDTR = 5;
  DMA1_Channel1->CPAR  = (uint32_t) & (ADC1->DR);
  DMA1_Channel1->CMAR  = (uint32_t)&adc_buffer;
  DMA1_Channel1->CCR   = DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1 | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_TCIE;
  DMA1_Channel1->CCR |= DMA_CCR_EN;

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/* ADC2 init function */
void MX_ADC2_Init(void) {
  ADC_ChannelConfTypeDef sConfig;

  __HAL_RCC_ADC2_CLK_ENABLE();

  // HAL_ADC_DeInit(&hadc2);
  // hadc2.Instance->CR2 = 0;
  /**Common config
    */
  hadc2.Instance                   = ADC2;
  hadc2.Init.ScanConvMode          = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode    = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion       = 5;
  HAL_ADC_Init(&hadc2);


  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.Channel = ADC_CHANNEL_10;  // pc0 right cur   -> left
  sConfig.Rank    = 1;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

  // sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  sConfig.Channel = ADC_CHANNEL_13;  // pc3 right b   -> left
  sConfig.Rank    = 2;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

  sConfig.Channel = ADC_CHANNEL_15;  // pc5 left c   -> right
  sConfig.Rank    = 3;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

  sConfig.Channel = ADC_CHANNEL_2;  // pa2 uart-l-tx
  sConfig.Rank    = 4;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  sConfig.Channel = ADC_CHANNEL_3;  // pa3 uart-l-rx
  sConfig.Rank    = 5;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

  hadc2.Instance->CR2 |= ADC_CR2_DMA;
  __HAL_ADC_ENABLE(&hadc2);
}

void receiveCommand(void)
{
    //HAL_UART_Receive_DMA(&huart3, (uint8_t *)&command, sizeof(command));

    if (command.start == SERIAL_START_FRAME && command.checksum == (uint16_t)calculateChecksumCommand(command)) {
        if (timeoutFlagSerial) {                      // Check for previous timeout flag
            if (timeoutCntSerial-- <= 0)                // Timeout de-qualification
                timeoutFlagSerial = 0;                    // Timeout flag cleared
        } else {
            leftMotor.settings = command.leftMotorSettings;
            rightMotor.settings = command.rightMotorSettings;

            if(command.overrideMotorInput)
            {
                leftMotor.input = command.leftMotorInput;
                rightMotor.input = command.rightMotorInput;
            }

            command.start     = 0xFFFF;                 // Change the Start Frame for timeout detection in the next cycle
            timeoutCntSerial  = 0;                      // Reset the timeout counter
        }
    } else {
        if (timeoutCntSerial++ >= SERIAL_TIMEOUT) {   // Timeout qualification
            //timeoutFlagSerial = 1;                      // Timeout detected
            //longBeep(8);
            timeoutCntSerial  = SERIAL_TIMEOUT;         // Limit timout counter value
        }
        // Most probably we are out-of-sync. Try to re-sync by reseting the DMA
        if (main_loop_counter % 30 == 0) {
            HAL_UART_DMAStop(&huart3);
            HAL_UART_Receive_DMA(&huart3, (uint8_t *)&command, sizeof(command));
        }
    }


    if (timeoutFlagSerial) {                          // In case of timeout bring the system to a Safe State
        leftMotor.settings.controlMode  = OpenMode;
        rightMotor.settings.controlMode  = OpenMode;  // OPEN_MODE request. This will bring the motor power to 0 in a controlled way

        leftMotor.input.pwm        = 0;
        rightMotor.input.pwm        = 0;
    }

    timeout = 0;
}
