/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Servo + Dual DC Motor (L293D) + Dual HC-SR04 for LEGO sorter
  *
  * Peripherals used:
  *   TIM1            : Free-running 1 µs counter (HC-SR04 timebase)
  *   TIM2 CH1 → PA0  : Servo PWM        (~50 Hz)
  *   TIM3 CH1 → PA6  : Motor 1 PWM (EN1) (~1 kHz)
  *   TIM3 CH2 → PA7  : Motor 2 PWM (EN2) (~1 kHz)
  *   PB0, PB1        : Motor 1 DIR1, DIR2 (L293D IN1, IN2)
  *   PB4, PB5        : Motor 2 DIR1, DIR2 (L293D IN3, IN4)
  *   PB6             : Sensor 1 TRIG (output)
  *   PB8             : Sensor 1 ECHO (input)  — 5V-tolerant pin
  *   PB9             : Sensor 2 TRIG (output)
  *   PB10            : Sensor 2 ECHO (input)  — 5V-tolerant pin
  *   USART2          : Debug UART (115200 baud)
  *
  * L293D wiring:
  *   Pin 1  (EN1) ← PA6  (TIM3_CH1)   Motor 1 enable/speed
  *   Pin 2  (IN1) ← PB0               Motor 1 direction A
  *   Pin 7  (IN2) ← PB1               Motor 1 direction B
  *   Pins 3 & 6   → Motor 1 terminals
  *
  *   Pin 9  (EN2) ← PA7  (TIM3_CH2)   Motor 2 enable/speed
  *   Pin 10 (IN3) ← PB4               Motor 2 direction A
  *   Pin 15 (IN4) ← PB5               Motor 2 direction B
  *   Pins 11 & 14 → Motor 2 terminals
  *
  *   Pin 8  (VS)        → 12 V
  *   Pin 16 (VSS)       → 5 V
  *   Pins 4,5,12,13     → GND
  *
  * HC-SR04 wiring:
  *   Sensor 1: VCC→5V  GND→GND  TRIG→PB9  ECHO→PB8  (via 1kΩ/2kΩ divider) Funnel
  *   Sensor 2: VCC→5V  GND→GND  TRIG→PB6  ECHO→PB10 (via 1kΩ/2kΩ divider) Camera
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ── UART helper ───────────────────────────────────────────────────────── */
static void uart_print(const char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}


static int32_t servo_current_angle = 0;  // software-tracked position

/*
 * Move to an absolute angle using standard RC servo PWM.
 * target_angle: degrees, clamped to 0-180.
 *
 * Assumes htim2/TIM_CHANNEL_1 is configured for 50 Hz PWM
 * (20 ms period) with a 1 MHz counter clock (1 tick = 1 µs),
 * so the compare value can be written directly in microseconds.
 * If your timer's tick isn't 1 µs, scale pulse_us accordingly.
 */
#define SERVO_MIN_PULSE_US   30   // pulse width at 0°
#define SERVO_MAX_PULSE_US  130   // pulse width at 180°

void Servo_SetAngle(int32_t target_angle)
{
    if (target_angle < 0)   target_angle = 0;
    if (target_angle > 180) target_angle = 180;

    uint32_t pulse_us = SERVO_MIN_PULSE_US +
        ((uint32_t)target_angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / 180;

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_us);

    char buf[48];
    sprintf(buf, "[SERVO] -> %ld deg (pulse=%lu us)\r\n", target_angle, pulse_us);
    uart_print(buf);

    servo_current_angle = target_angle;
}
/* ── DC Motors via L293D ───────────────────────────────────────────────── */

typedef enum {
    MOTOR_FORWARD  = 0,
    MOTOR_BACKWARD = 1,
    MOTOR_STOP     = 2
} MotorDir_t;

typedef enum {
    MOTOR_1 = 1,
    MOTOR_2 = 2
} MotorID_t;

/**
 * @brief  Set speed and direction for a given motor.
 * @param  motor  MOTOR_1 or MOTOR_2
 * @param  speed  0-100 % duty cycle
 * @param  dir    MOTOR_FORWARD | MOTOR_BACKWARD | MOTOR_STOP
 *
 * TIM3 ARR = 999  ->  CCR = speed * 10  (0-100% maps to 0-1000)
 */
void Motor_Set(MotorID_t motor, uint8_t speed, MotorDir_t dir)
{
    if (speed > 100) speed = 100;

    /* Direction pins and timer channel differ per motor */
    GPIO_TypeDef *port_a, *port_b;
    uint16_t      pin_a,   pin_b;
    uint32_t      tim_ch;

    if (motor == MOTOR_1) {
        port_a = GPIOB; pin_a = GPIO_PIN_0;   // IN1
        port_b = GPIOB; pin_b = GPIO_PIN_1;   // IN2
        tim_ch = TIM_CHANNEL_1;               // EN1 on PA6
    } else {
        port_a = GPIOB; pin_a = GPIO_PIN_4;   // IN3
        port_b = GPIOB; pin_b = GPIO_PIN_5;   // IN4
        tim_ch = TIM_CHANNEL_2;               // EN2 on PA7
    }

    switch (dir) {
        case MOTOR_FORWARD:
            HAL_GPIO_WritePin(port_a, pin_a, GPIO_PIN_SET);
            HAL_GPIO_WritePin(port_b, pin_b, GPIO_PIN_RESET);
            break;
        case MOTOR_BACKWARD:
            HAL_GPIO_WritePin(port_a, pin_a, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(port_b, pin_b, GPIO_PIN_SET);
            break;
        case MOTOR_STOP:
        default:
            HAL_GPIO_WritePin(port_a, pin_a, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(port_b, pin_b, GPIO_PIN_RESET);
            speed = 0;
            break;
    }

    uint32_t ccr = (uint32_t)speed * 10;
    __HAL_TIM_SET_COMPARE(&htim3, tim_ch, ccr);

    const char *dir_str = (dir == MOTOR_FORWARD)  ? "FWD" :
                          (dir == MOTOR_BACKWARD)  ? "BWD" : "STOP";
    char buf[56];
    sprintf(buf, "[MOTOR %d] dir=%-4s  speed=%3d%%  CCR=%lu\r\n",
            (int)motor, dir_str, speed, ccr);
    uart_print(buf);
}

/* ── HC-SR04 ultrasonic distance sensors ──────────────────────────────── */

#define HCSR04_TIMEOUT_US  38000u   // ~38 ms = no object in range

typedef enum { HCSR04_1 = 0, HCSR04_2 = 1 } HCSR04_ID_t;

static const struct {
    GPIO_TypeDef *trig_port; uint16_t trig_pin;
    GPIO_TypeDef *echo_port; uint16_t echo_pin;
} hcsr04_map[2] = {
    { GPIOB, GPIO_PIN_9,  GPIOB, GPIO_PIN_8  },   // Sensor 1
    { GPIOB, GPIO_PIN_6,  GPIOB, GPIO_PIN_10 },   // Sensor 2
};

/* Returns current TIM1 counter value (1 tick = 1 µs) */
static inline uint16_t micros(void)
{
    return (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
}

/**
 * @brief  Trigger one measurement and return distance in mm.
 * @param  id   HCSR04_1 or HCSR04_2
 * @retval Distance in mm, or 0 if no echo within timeout (~6.5 m).
 */
uint32_t HCSR04_Read_mm(HCSR04_ID_t id)
{
    GPIO_TypeDef *tp   = hcsr04_map[id].trig_port;
    uint16_t      tpin = hcsr04_map[id].trig_pin;
    GPIO_TypeDef *ep   = hcsr04_map[id].echo_port;
    uint16_t      epin = hcsr04_map[id].echo_pin;

    /* 10 µs trigger pulse */
    HAL_GPIO_WritePin(tp, tpin, GPIO_PIN_SET);
    uint16_t t = micros();
    while ((uint16_t)(micros() - t) < 10u) {}
    HAL_GPIO_WritePin(tp, tpin, GPIO_PIN_RESET);

    /* Wait for ECHO to go high */
    t = micros();
    while (HAL_GPIO_ReadPin(ep, epin) == GPIO_PIN_RESET) {
        if ((uint16_t)(micros() - t) >= HCSR04_TIMEOUT_US) return 0u;
    }

    /* Measure how long ECHO stays high */
    uint16_t start = micros();
    while (HAL_GPIO_ReadPin(ep, epin) == GPIO_PIN_SET) {
        if ((uint16_t)(micros() - start) >= HCSR04_TIMEOUT_US) return 0u;
    }
    uint16_t duration_us = (uint16_t)(micros() - start);

    /* distance (mm) = duration_µs × 0.1715  ≈  duration × 343 / 2000 */
    return ((uint32_t)duration_us * 343UL) / 2000UL;
}


//new stuff

// Example threshold; you said you’ll set it. Keep as a define.
#define TRIGGER_MM  120  // <-- change

// Minimum time that must pass between two camera (CMRA) triggers, even if
// the object briefly clears the sensor window and re-enters (sensor jitter)
// or a second brick arrives right behind the first. Tune to taste.
#define CMRA_MIN_INTERVAL_MS  10000u   // <-- change




/* ===================== UART (Pi link) — simplified ===================== */
/*
 * Minimal send/receive API for talking to the Raspberry Pi over USART1.
 * Messages are newline-terminated strings, e.g. "RESULT 3 OK 2x4 Bricks\n".
 *
 *   Pi_UART_StartReceive()             — call once at startup (and again
 *                                         after a UART error) to arm reception.
 *   Pi_UART_ReceiveMessage(buf, sz)    — poll from your main loop; returns
 *                                         true and fills buf when a new
 *                                         newline-terminated message has
 *                                         arrived (without the '\n').
 *   Pi_UART_SendMessage(msg)           — send a string to the Pi. Include
 *                                         your own trailing '\n' if your
 *                                         protocol expects line framing.
 *
 * Reception uses DMA + idle-line detection so the main loop never blocks
 * waiting on bytes; build whatever command parsing / funnel control you
 * need on top of Pi_UART_ReceiveMessage().
 */

#define PI_RX_DMA_BUF_SZ   256
#define PI_LINE_BUF_SZ     256

static uint8_t  pi_rx_dma_buf[PI_RX_DMA_BUF_SZ];
static uint16_t pi_rx_old_pos = 0;

static char     pi_accum_buf[PI_LINE_BUF_SZ];   // line currently being assembled
static uint16_t pi_accum_len = 0;

static char              pi_line_buf[PI_LINE_BUF_SZ]; // last completed line
static uint16_t          pi_line_len = 0;
static volatile bool     pi_line_ready = false;

/* Internal: feed raw bytes from the DMA buffer into the line assembler */
static void pi_accumulate_bytes(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        uint8_t c = data[i];

        if (c == '\r') continue;          // ignore CR

        if (c == '\n') {                  // end of message
            if (pi_accum_len > 0 && !pi_line_ready) {
                memcpy(pi_line_buf, pi_accum_buf, pi_accum_len);
                pi_line_buf[pi_accum_len] = 0;
                pi_line_len   = pi_accum_len;
                pi_line_ready = true;     // consumed by Pi_UART_ReceiveMessage()
            }
            pi_accum_len = 0;
            continue;
        }

        if (pi_accum_len < (PI_LINE_BUF_SZ - 1)) {
            pi_accum_buf[pi_accum_len++] = c;
        } else {
            pi_accum_len = 0;             // overflow: drop the line
        }
    }
}

/**
 * @brief  Start (or restart) listening for messages from the Pi.
 *         Call once at startup, and again after a UART error.
 */
void Pi_UART_StartReceive(void)
{
    pi_rx_old_pos = 0;
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, pi_rx_dma_buf, sizeof(pi_rx_dma_buf)) != HAL_OK) {
        uart_print("[PI] HAL_UARTEx_ReceiveToIdle_DMA failed\r\n");
    }
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

/**
 * @brief  Poll for a newly received message from the Pi.
 * @param  out     buffer to copy the message into (null-terminated, no '\n')
 * @param  out_sz  size of out
 * @retval true if a new message was copied into out, false if nothing new
 */
bool Pi_UART_ReceiveMessage(char *out, uint16_t out_sz)
{
    if (!pi_line_ready) return false;

    uint16_t n = pi_line_len;
    if (n >= out_sz) n = out_sz - 1;
    memcpy(out, pi_line_buf, n);
    out[n] = 0;

    pi_line_ready = false;
    return true;
}

/**
 * @brief  Send a message to the Pi.
 */
void Pi_UART_SendMessage(const char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
}

/* ── Funnel Command Queue ─────────────────────────────────────────────────── */

#define FUNNEL_QUEUE_SIZE  16

typedef struct {
    char     items[FUNNEL_QUEUE_SIZE][32];
    uint8_t  head;
    uint8_t  tail;
    uint8_t  count;
} FunnelQueue;

static FunnelQueue fq = {0};

static bool FQ_Enqueue(const char *cmd)
{
    if (fq.count >= FUNNEL_QUEUE_SIZE) return false;
    strncpy(fq.items[fq.tail], cmd, 31);
    fq.items[fq.tail][31] = '\0';
    fq.tail = (fq.tail + 1) % FUNNEL_QUEUE_SIZE;
    fq.count++;
    return true;
}

static bool FQ_Dequeue(char *out)
{
    if (fq.count == 0) return false;
    strncpy(out, fq.items[fq.head], 31);
    fq.head = (fq.head + 1) % FUNNEL_QUEUE_SIZE;
    fq.count--;
    return true;
}

static const char *FQ_Peek(void)
{
    if (fq.count == 0) return NULL;
    return fq.items[fq.head];
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);              // free-running µs counter (HC-SR04)
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);   // Servo  — PA0
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);   // Motor1 — PA6
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);   // Motor2 — PA7

  uart_print("\r\n=== LEGO Sorter - Dual Motor Test ===\r\n");

  Pi_UART_StartReceive();
  Pi_UART_SendMessage("EVT READY\n");

  /* ── Servo ──────────────────────────────────────────── */

  //Servo_SetAngle(0); 		HAL_Delay(5000); //- Bin 1, 1x1
  //Servo_SetAngle(37);   	HAL_Delay(5000); //- Bin 2, 1x2
  //Servo_SetAngle(74);   	HAL_Delay(5000); //- Bin 3, 1x4
  //Servo_SetAngle(111);  	HAL_Delay(5000); //- Bin 4, 2x2
  //Servo_SetAngle(148);   	HAL_Delay(5000); //- Bin 5, 2x4
  //Servo_SetAngle(180);   	HAL_Delay(5000); //- Bin 6, err
  //Servo_SetAngle(0); 		HAL_Delay(5000);

  /* ── Motors ───────────────────────────────────────────────── */

  Motor_Set(MOTOR_1, 100, MOTOR_FORWARD);
  //HAL_Delay(5000);
  Motor_Set(MOTOR_2, 85, MOTOR_BACKWARD);
  //HAL_Delay(5000);

  //Motor_Set(MOTOR_1,  0, MOTOR_STOP);
  //Motor_Set(MOTOR_2, 0, MOTOR_STOP);
  //HAL_Delay(1000);

  uart_print("\r\n=== Test complete ===\r\n");

  char msg[PI_LINE_BUF_SZ];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* ── state that persists across iterations ── */
  static bool     cmra_triggered  = false;   // S2 camera debounce
  static bool     s1_triggered    = false;   // S1 funnel debounce
  static char     last_applied[32] = "";     // last servo position
  static bool     defaulted        = false;  // idle default guard
  static uint32_t last_cmra_trigger_ms = (uint32_t)(0 - CMRA_MIN_INTERVAL_MS);
                                              // ^ pre-loaded so the very first
                                              //   detection isn't held off by
                                              //   the cooldown (wraparound-safe)

  while (1)
  {
      /* ═══════════════════════════════════════════════════════════════
       * 1. SONAR
       * ═══════════════════════════════════════════════════════════════ */
      uint32_t d1 = HCSR04_Read_mm(HCSR04_1);   // funnel / bin sensor
      uint32_t d2 = HCSR04_Read_mm(HCSR04_2);   // camera sensor

      char buf[64];
      snprintf(buf, sizeof(buf), "[SONAR] S1=%lu mm  S2=%lu mm\r\n", d1, d2);
      uart_print(buf);

      /* ═══════════════════════════════════════════════════════════════
       * 2. CAMERA TRIGGER  (S2: brick in front of camera)
       *    Fire CMRA once per brick, wait for Pi reply, enqueue it.
       *    A minimum-interval cooldown (CMRA_MIN_INTERVAL_MS) additionally
       *    guards against a jittery reading re-triggering on the same
       *    brick or a second brick arriving too soon behind the first.
       * ═══════════════════════════════════════════════════════════════ */
      if (d2 >= 60 && d2 <= 180)
      {
          if (!cmra_triggered)
          {
              uint32_t now = HAL_GetTick();

              if ((uint32_t)(now - last_cmra_trigger_ms) >= CMRA_MIN_INTERVAL_MS)
              {
                  cmra_triggered = true;
                  last_cmra_trigger_ms = now;

                  uart_print("[CAM] Brick detected — sending CMRA\r\n");
                  msg[0] = '\0';                 // clear any stale reply from a previous cycle
                  Pi_UART_SendMessage("CMRA\n");

                  /* Block until Pi replies or 5 s timeout */
                  uint32_t wait_start = HAL_GetTick();
                  bool got_reply = false;

                  while (HAL_GetTick() - wait_start < 5000)
                  {
                      if (Pi_UART_ReceiveMessage(msg, sizeof(msg)))
                      {
                          got_reply = true;
                          break;
                      }
                      HAL_Delay(10);
                  }

                  if (!got_reply)
                  {
                      /* No reply within the timeout = no piece detected.
                       * Don't enqueue anything — just go back to waiting
                       * for the next brick. */
                      uart_print("[CAM] No reply from Pi — no piece detected, ignoring\r\n");
                  }
                  else
                  {
                      /* Validate and enqueue */
                      bool known = (strcmp(msg, "1x1 Bricks") == 0 ||
                                    strcmp(msg, "1x2 Bricks") == 0 ||
                                    strcmp(msg, "1x4 Bricks") == 0 ||
                                    strcmp(msg, "2x2 Bricks") == 0 ||
                                    strcmp(msg, "2x4 Bricks") == 0 ||
                                    strcmp(msg, "ERR")         == 0);

                      if (known)
                      {
                          if (FQ_Enqueue(msg))
                          {
                              char log[64];
                              snprintf(log, sizeof(log),
                                       "[QUEUE] Enqueued: '%s'  (%u in queue)\r\n",
                                       msg, fq.count);
                              uart_print(log);
                              defaulted = false;   // queue is live again
                          }
                          else
                          {
                              uart_print("[WARN] Queue full — command dropped\r\n");
                              Pi_UART_SendMessage("EVT QUEUE_FULL\n");
                          }
                      }
                      else
                      {
                          char warn[PI_LINE_BUF_SZ + 32];
                          snprintf(warn, sizeof(warn), "[WARN] Unknown Pi reply: '%s'\r\n", msg);
                          uart_print(warn);
                      }
                  }
              }
              /* else: still cooling down since the last trigger — ignore
               * this reading, without setting cmra_triggered, so we'll
               * re-check as soon as the cooldown elapses. */
          }
          // brick still in window — wait for it to leave
      }
      else
      {
          cmra_triggered = false;   // brick cleared camera, ready for next
      }

      /* ═══════════════════════════════════════════════════════════════
       * 3. APPLY QUEUE FRONT TO SERVO
       *    Only moves when the front of the queue changes.
       * ═══════════════════════════════════════════════════════════════ */
      const char *front = FQ_Peek();

      if (front != NULL)
      {
          if (strcmp(front, last_applied) != 0)
          {
              if      (strcmp(front, "1x1 Bricks") == 0) Servo_SetAngle(0);
              else if (strcmp(front, "1x2 Bricks") == 0) Servo_SetAngle(37);
              else if (strcmp(front, "1x4 Bricks") == 0) Servo_SetAngle(74);
              else if (strcmp(front, "2x2 Bricks") == 0) Servo_SetAngle(111);
              else if (strcmp(front, "2x4 Bricks") == 0) Servo_SetAngle(148);
              else if (strcmp(front, "ERR")         == 0) Servo_SetAngle(180);

              strncpy(last_applied, front, sizeof(last_applied) - 1);
              Pi_UART_SendMessage("EVT SORTED\n");

              char log[64];
              snprintf(log, sizeof(log), "[SERVO] Applied: '%s'\r\n", front);
              uart_print(log);
          }
      }
      else if (!defaulted)
      {
          /* Queue exhausted — park at 1x1 */
          uart_print("[QUEUE] Empty — returning to 1x1\r\n");
          Servo_SetAngle(0);
          last_applied[0] = '\0';   // clear so next queue fill applies fresh
          defaulted = true;
      }

      /* ═══════════════════════════════════════════════════════════════
       * 4. FUNNEL SENSOR DEQUEUE  (S1: brick passing through funnel)
       *    Dequeue once per brick crossing the 120–160 mm window.
       * ═══════════════════════════════════════════════════════════════ */
      if (d1 >= 120 && d1 <= 160)
      {
          if (!s1_triggered)
          {
              s1_triggered = true;

              char popped[32];
              if (FQ_Dequeue(popped))
              {
                  char log[64];
                  snprintf(log, sizeof(log),
                           "[QUEUE] Dequeued: '%s'  (%u remaining)\r\n",
                           popped, fq.count);
                  uart_print(log);

                  /* Wait for the piece to fall into the bin before moving the funnel */
                  HAL_Delay(10000);   // ← tune this; start high, reduce until reliable
              }
              else
              {
                  uart_print("[QUEUE] Dequeue on empty queue — ignored\r\n");
              }
          }
      }
      else
      {
          s1_triggered = false;
      }
      HAL_Delay(60);
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1679;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB4 PB5
                           PB6 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* Called by HAL on UART “Receive To Idle DMA” events */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART1) return;

    // Size = number of bytes currently in the DMA buffer (0..PI_RX_DMA_BUF_SZ)
    if (Size != pi_rx_old_pos) {
        if (Size > pi_rx_old_pos) {
            // Linear new data
            pi_accumulate_bytes(&pi_rx_dma_buf[pi_rx_old_pos], Size - pi_rx_old_pos);
        } else {
            // Wrapped around (circular DMA)
            pi_accumulate_bytes(&pi_rx_dma_buf[pi_rx_old_pos], PI_RX_DMA_BUF_SZ - pi_rx_old_pos);
            if (Size > 0) {
                pi_accumulate_bytes(&pi_rx_dma_buf[0], Size);
            }
        }
        pi_rx_old_pos = Size;
        if (pi_rx_old_pos == PI_RX_DMA_BUF_SZ) pi_rx_old_pos = 0;
    }

    // Re-arm reception (recommended pattern with ReceiveToIdle)
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, pi_rx_dma_buf, PI_RX_DMA_BUF_SZ);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        // Attempt to recover RX
        Pi_UART_StartReceive();
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
