/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "tim.h"
#include "bsp_ssd1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* ESP32-S3 帧解析出的目标状态：唯一写者是 esp32linkTask，其余任务只读 */
typedef struct {
  int16_t  dx, dy;      /* 人脸中心相对画面中心的偏差（像素，右/下为正） */
  uint16_t dist_mm;     /* 单目测距 mm，0=无效 */
  uint8_t  has_target;  /* 0=无目标 1=有目标 */
  uint32_t last_ms;     /* 最近一次收到帧的时刻 */
} target_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_LINE_LEN 48     /* 单帧缓冲：$000,000,320,240,# 约 18 字节 */
#define CAM_W 320          /* 模块检测分辨率（亚博 AI 固件 QVGA，实测确认） */
#define CAM_H 240
#define CAM_CENTER_X (CAM_W / 2)   /* 画面中心 160 */
#define CAM_CENTER_Y (CAM_H / 2)   /* 画面中心 120 */
#define FACE_DIST_K 68800  /* 单目测距系数，2026-09-07 标定：400mm × 框宽172px（脸距镜头40cm实测） */

/* 云台（SG90 ×2：TIM4 CH1=PD12 水平 / CH2=PD13 俯仰，50Hz，脉宽单位 µs） */
#define GIMBAL_PERIOD_MS 20
#define SERVO_PWM_MIN    500u
#define SERVO_PWM_MAX    2500u
#define SERVO_PWM_CENTER 1500u
#define GIMBAL_DEADZONE  8      /* 像素死区：人脸偏移小于此值不动，防抖 */
#define GIMBAL_KP        1.0f   /* 增量式 P：偏多少像素修多少 */
#define GIMBAL_KD        2.0f   /* D 项抑制超调 */
#define GIMBAL_SLEW      60     /* 每周期最大脉宽变化 µs（60/20ms → 满摆约 0.7s，动作柔和） */
#define GIMBAL_LOST_MS   1000   /* 丢目标超过 1s → 缓慢回中 */
#define PAN_INVERT       0      /* 装好后方向反了就改成 1 */
#define TILT_INVERT      0

/* OneNET 上云（新版 MQTT 物模型；要求 ESP8266 AT 固件 ≥ v2.0，用 AT+GMR 查版本）
 * OneNET 控制台操作：创建产品(选MQTT) → 功能定义加 person/dist 两个功能点 → 添加设备
 * → 把 产品ID/设备名称/token 填到下面三个宏（token 用官方 Token 生成工具做） */
#define ONENET_WIFI_SSID  "your_ssid"
#define ONENET_WIFI_PASS  "your_password"
#define ONENET_MQTT_HOST  "mqtts.heclouds.com"
#define ONENET_MQTT_PORT  1883
#define ONENET_PRODUCT_ID "your_product_id"    /* OneNET 产品ID */
#define ONENET_DEVICE_NAME "your_device_name"  /* OneNET 设备名称 */
#define ONENET_TOKEN      "your_token"         /* 设备 token（平台工具生成） */
#define ONENET_REPORT_MS  5000u                /* 上报周期 ms */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
volatile target_t g_target;
volatile uint16_t g_dist_filtered = 0;  /* distTask 滑动平均后的距离，OLED/上报读这个 */
volatile uint8_t  g_person_seen = 0;    /* 当前画面有没有人（含 1s 链路超时判断），OLED/上报读这个 */
uint8_t rx_byte;                        /* USART3 单字节接收缓冲（摄像头模块帧） */
uint8_t rx2_byte;                       /* USART2 单字节接收缓冲（DAPLink 控制台指令转发） */
osMessageQueueId_t uart2_rx_qHandle;    /* USART2 收到的字节进这里，由 esp32linkTask 转发给模块 */
static char    rx_line[RX_LINE_LEN];
static uint8_t rx_pos = 0;
static uint8_t rx_capturing = 0;        /* '$' 开始、'#' 结束的亚博帧捕获标志 */
/* USER CODE END Variables */
/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for esp32linkTask */
osThreadId_t esp32linkTaskHandle;
const osThreadAttr_t esp32linkTask_attributes = {
  .name = "esp32linkTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for gimbalTask */
osThreadId_t gimbalTaskHandle;
const osThreadAttr_t gimbalTask_attributes = {
  .name = "gimbalTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for distTask */
osThreadId_t distTaskHandle;
const osThreadAttr_t distTask_attributes = {
  .name = "distTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for oledTask */
osThreadId_t oledTaskHandle;
const osThreadAttr_t oledTask_attributes = {
  .name = "oledTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for esp8266Task */
osThreadId_t esp8266TaskHandle;
const osThreadAttr_t esp8266Task_attributes = {
  .name = "esp8266Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for uart3_rx_q */
osMessageQueueId_t uart3_rx_qHandle;
const osMessageQueueAttr_t uart3_rx_q_attributes = {
  .name = "uart3_rx_q"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void parse_line(const char *s);
static int  servo_clamp_slew(int cur, int cmd);
static void esp8266_send(const char *s);
static int  esp8266_wait(const char *expect, uint32_t timeout_ms);
/* USER CODE END FunctionPrototypes */

void ledTask1(void *argument);
void esp32linkTask1(void *argument);
void gimbalTask1(void *argument);
void distTask1(void *argument);
void oledTask1(void *argument);
void esp8266Task1(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of uart3_rx_q */
  uart3_rx_qHandle = osMessageQueueNew (64, sizeof(uint8_t), &uart3_rx_q_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* DAPLink 控制台(USART2)→模块 指令转发队列（调试用：wifi_ver / ai_mode:2 等） */
  uart2_rx_qHandle = osMessageQueueNew(64, sizeof(uint8_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ledTask */
  ledTaskHandle = osThreadNew(ledTask1, NULL, &ledTask_attributes);

  /* creation of esp32linkTask */
  esp32linkTaskHandle = osThreadNew(esp32linkTask1, NULL, &esp32linkTask_attributes);

  /* creation of gimbalTask */
  gimbalTaskHandle = osThreadNew(gimbalTask1, NULL, &gimbalTask_attributes);

  /* creation of distTask */
  distTaskHandle = osThreadNew(distTask1, NULL, &distTask_attributes);

  /* creation of oledTask */
  oledTaskHandle = osThreadNew(oledTask1, NULL, &oledTask_attributes);

  /* creation of esp8266Task */
  esp8266TaskHandle = osThreadNew(esp8266Task1, NULL, &esp8266Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_ledTask1 */
/**
  * @brief  Function implementing the ledTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ledTask1 */
void ledTask1(void *argument)
{
  /* USER CODE BEGIN ledTask1 */
  /* 板载用户 LED 在 PB2，先把它配成推挽输出 */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef led_init = {0};
  led_init.Pin = GPIO_PIN_2;
  led_init.Mode = GPIO_MODE_OUTPUT_PP;
  led_init.Pull = GPIO_NOPULL;
  led_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &led_init);
  /* Infinite loop */
  for(;;)
  {
   HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);   // 板载绿灯心跳：每 500ms 翻转
   printf("rader alive, tick=%lu\r\n", HAL_GetTick());
   osDelay(500);
  }
  /* USER CODE END ledTask1 */
}

/* USER CODE BEGIN Header_esp32linkTask1 */
/**
* @brief Function implementing the esp32linkTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_esp32linkTask1 */
void esp32linkTask1(void *argument)
{
  /* USER CODE BEGIN esp32linkTask1 */
  /* 调度器、队列就绪后才启动串口接收中断（铁律：别放 main 里） */
  HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
  HAL_UART_Receive_IT(&huart2, &rx2_byte, 1);
  /* 保险：等模块上电稳定后自动切人脸检测模式（已在模式则只回 OK，无害） */
  osDelay(3000);
  {
    static const char cmd[] = "ai_mode:2";
    HAL_UART_Transmit(&huart3, (uint8_t *)cmd, sizeof(cmd) - 1, 100);
  }
  for (;;)
  {
    uint8_t b;
    /* --- 1. 把 USART3 队列一次清空（帧字节连发，不能一个循环只吃一个） --- */
    while (osMessageQueueGet(uart3_rx_qHandle, &b, NULL, 0) == osOK)
    {
      if (b == '$')
      {
        rx_pos = 0;
        rx_capturing = 1;
      }
      else if (rx_capturing)
      {
        if (b == '#')
        {
          rx_line[rx_pos] = '\0';
          parse_line(rx_line);
          printf("[RX] %s\r\n", rx_line);   /* 调试用：收到什么打什么，验证后可删 */
          rx_capturing = 0;
        }
        else if (rx_pos < RX_LINE_LEN - 1)
        {
          rx_line[rx_pos++] = (char)b;
        }
        else
        {
          rx_capturing = 0;                 /* 超长脏帧，丢弃重新同步 */
        }
      }
    }
    /* --- 2. DAPLink 控制台(USART2)输入 → 原样转发给模块（非阻塞看一眼） --- */
    if (osMessageQueueGet(uart2_rx_qHandle, &b, NULL, 0) == osOK)
    {
      HAL_UART_Transmit(&huart3, &b, 1, 20);
    }
    osDelay(1);   /* 1ms 巡检一圈：115200 下 1ms 最多来 ~12 字节，64 深队列绰绰有余 */
  }
  /* USER CODE END esp32linkTask1 */
}

/* USER CODE BEGIN Header_gimbalTask1 */
/**
* @brief Function implementing the gimbalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gimbalTask1 */
void gimbalTask1(void *argument)
{
  /* USER CODE BEGIN gimbalTask1 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);   /* PD12 水平舵机 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);   /* PD13 俯仰舵机 */
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, SERVO_PWM_CENTER);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, SERVO_PWM_CENTER);

  float   pan = SERVO_PWM_CENTER, tilt = SERVO_PWM_CENTER;
  int     pan_out = SERVO_PWM_CENTER, tilt_out = SERVO_PWM_CENTER;
  int16_t prev_dx = 0, prev_dy = 0;
  const uint32_t period = GIMBAL_PERIOD_MS;
  uint32_t last = osKernelGetTickCount();

  for (;;)
  {
    int16_t dx = g_target.dx, dy = g_target.dy;
    int has = g_target.has_target &&
              (HAL_GetTick() - g_target.last_ms < GIMBAL_LOST_MS);

    if (has)
    {
      if (dx > -GIMBAL_DEADZONE && dx < GIMBAL_DEADZONE) dx = 0;
      if (dy > -GIMBAL_DEADZONE && dy < GIMBAL_DEADZONE) dy = 0;
#if PAN_INVERT
      dx = -dx;
#endif
#if TILT_INVERT
      dy = -dy;
#endif
      /* 增量式 PD：人脸每偏一个像素，本轮就朝那边修一点 */
      pan  += GIMBAL_KP * dx + GIMBAL_KD * (dx - prev_dx);
      tilt += GIMBAL_KP * dy + GIMBAL_KD * (dy - prev_dy);
      prev_dx = dx;
      prev_dy = dy;
    }
    else
    {
      /* 丢目标：缓慢回中，不猛甩 */
      if (pan  > SERVO_PWM_CENTER) pan  -= 5; else if (pan  < SERVO_PWM_CENTER) pan  += 5;
      if (tilt > SERVO_PWM_CENTER) tilt -= 5; else if (tilt < SERVO_PWM_CENTER) tilt += 5;
      prev_dx = 0;
      prev_dy = 0;
    }

    pan_out  = servo_clamp_slew(pan_out,  (int)pan);
    tilt_out = servo_clamp_slew(tilt_out, (int)tilt);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (uint32_t)pan_out);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, (uint32_t)tilt_out);

    last += period;
    osDelayUntil(last);
  }
  /* USER CODE END gimbalTask1 */
}

/* USER CODE BEGIN Header_distTask1 */
/**
* @brief Function implementing the distTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_distTask1 */
void distTask1(void *argument)
{
  /* USER CODE BEGIN distTask1 */
  const uint32_t period = 50;                 /* 50ms 滤波周期 */
  uint32_t last = osKernelGetTickCount();
  static uint16_t buf[8];
  static uint8_t  idx = 0, cnt = 0;
  for (;;)
  {
    if (g_target.has_target && g_target.dist_mm > 0)
    {
      buf[idx] = g_target.dist_mm;
      idx = (idx + 1) % 8;
      if (cnt < 8) cnt++;
      uint32_t sum = 0;
      for (uint8_t i = 0; i < cnt; i++) sum += buf[i];
      g_dist_filtered = (uint16_t)(sum / cnt);
    }
    else
    {
      g_dist_filtered = 0;                    /* 无目标/无效距离 → 清零 */
      idx = 0; cnt = 0;
    }
    /* "有没有人"：有目标 且 1 秒内还有新帧才算（防 ESP32 断链后残留旧状态） */
    g_person_seen = (g_target.has_target &&
                     (HAL_GetTick() - g_target.last_ms < 1000)) ? 1 : 0;
    last += period;
    osDelayUntil(last);
  }
  /* USER CODE END distTask1 */
}

/* USER CODE BEGIN Header_oledTask1 */
/**
* @brief Function implementing the oledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_oledTask1 */
void oledTask1(void *argument)
{
  /* USER CODE BEGIN oledTask1 */
  char line[16];
  SSD1306_Init();                       /* 屏没接也不会卡死：ok=0，循环里自动跳过 */
  for (;;)
  {
    if (SSD1306_IsOK())
    {
      uint16_t dist = g_dist_filtered;  /* 先拷贝，避免刷新中途被别的任务改掉 */
      SSD1306_Fill(0x00);
      SSD1306_DrawString(0, 0, "Person:", 1);
      SSD1306_DrawString(48, 0, g_person_seen ? "YES" : "NO", 1);
      if (dist > 0) sprintf(line, "%u mm", (unsigned)dist);
      else          sprintf(line, "----");
      SSD1306_DrawString(0, 18, line, 2);   /* 距离放大一倍，远处看得清 */
      SSD1306_Flush();
    }
    osDelay(100);                     /* 10Hz 刷新 */
  }
  /* USER CODE END oledTask1 */
}

/* USER CODE BEGIN Header_esp8266Task1 */
/**
* @brief Function implementing the esp8266Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_esp8266Task1 */
void esp8266Task1(void *argument)
{
  /* USER CODE BEGIN esp8266Task1 */
  /* OneNET 新版 MQTT 物模型上报。
   * 功能点需在 OneNET 产品里预先定义，标识符必须是 person 和 dist。 */
  int net_ok = 0, mqtt_ok = 0;
  char payload[96], topic[64], pub[192];
  osDelay(5000);                    /* 等 ESP8266 上电启动 */
  for (;;)
  {
    int person = g_person_seen;
    int dist    = g_dist_filtered;

    if (!net_ok)
    {
      esp8266_send("AT\r\n");
      if (!esp8266_wait("OK", 800)) { osDelay(2000); continue; }
      esp8266_send("AT+CWMODE=1\r\n");
      esp8266_wait("OK", 1000);
      snprintf(pub, sizeof(pub), "AT+CWJAP=\"%s\",\"%s\"\r\n",
               ONENET_WIFI_SSID, ONENET_WIFI_PASS);
      esp8266_send(pub);
      net_ok = esp8266_wait("WIFI GOT IP", 20000);
      if (!net_ok) { osDelay(3000); continue; }
    }

    if (!mqtt_ok)
    {
      snprintf(pub, sizeof(pub), "AT+MQTTUSERCFG=0,1,\"%s\",\"%s\",\"%s\",0,0,\"\"\r\n",
               ONENET_DEVICE_NAME, ONENET_PRODUCT_ID, ONENET_TOKEN);
      esp8266_send(pub);
      if (!esp8266_wait("OK", 3000)) { osDelay(2000); continue; }
      snprintf(pub, sizeof(pub), "AT+MQTTCONN=0,\"%s\",%d,1\r\n",
               ONENET_MQTT_HOST, ONENET_MQTT_PORT);
      esp8266_send(pub);
      mqtt_ok = esp8266_wait("+MQTTCONNECTED", 15000);
      if (!mqtt_ok)
      {
        esp8266_send("AT+MQTTCLEAN=0\r\n");
        esp8266_wait("OK", 1000);
        osDelay(3000);
        continue;
      }
    }

    /* 物模型属性上报：topic $sys/{产品ID}/{设备名}/dp/post/json，分三步拼装避免超长 */
    snprintf(payload, sizeof(payload),
             "{\"id\":\"1\",\"params\":{\"person\":{\"v\":%d},\"dist\":{\"v\":%d}}}",
             person, dist);
    snprintf(topic, sizeof(topic), "$sys/%s/%s/dp/post/json",
             ONENET_PRODUCT_ID, ONENET_DEVICE_NAME);
    snprintf(pub, sizeof(pub), "AT+MQTTPUB=0,\"%s\",\"%s\",1,0\r\n", topic, payload);
    esp8266_send(pub);
    if (!esp8266_wait("OK", 5000))  /* 发布失败 = 掉线，回到重连流程 */
    {
      mqtt_ok = 0;
      continue;
    }
    osDelay(ONENET_REPORT_MS);
  }
  /* USER CODE END esp8266Task1 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* 解析亚博人脸框：$x1,y1,x2,y2,#（左上角、右下角），算偏差+单目测距 */
static void parse_line(const char *s)
{
  int x1, y1, x2, y2;
  if (sscanf(s, "%d,%d,%d,%d", &x1, &y1, &x2, &y2) != 4) return;
  if (x1 == 0 && y1 == 0 && x2 >= CAM_W && y2 >= CAM_H)
  {
    /* 全画幅框 = 模块未检出人脸：立即置无人 */
    g_target.has_target = 0;
    g_target.dist_mm = 0;
    g_target.last_ms = HAL_GetTick();
    return;
  }
  int w = x2 - x1;
  g_target.dx = (int16_t)((x1 + x2) / 2 - CAM_CENTER_X);
  g_target.dy = (int16_t)((y1 + y2) / 2 - CAM_CENTER_Y);
  g_target.dist_mm = (w > 0) ? (uint16_t)(FACE_DIST_K / w) : 0;
  g_target.has_target = 1;
  g_target.last_ms = HAL_GetTick();
}

/* 串口收完一个字节：字节进队列 + 重新挂接收（中断里只做这两件事） */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    osMessageQueuePut(uart3_rx_qHandle, &rx_byte, 0, 0);
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
  }
  else if (huart->Instance == USART2)
  {
    osMessageQueuePut(uart2_rx_qHandle, &rx2_byte, 0, 0);
    HAL_UART_Receive_IT(&huart2, &rx2_byte, 1);
  }
}

/* 舵机脉宽限幅 + 变化限速：每周期最多动 GIMBAL_SLEW µs，动作柔和保护齿轮 */
static int servo_clamp_slew(int cur, int cmd)
{
  if (cmd < SERVO_PWM_MIN) cmd = SERVO_PWM_MIN;
  if (cmd > SERVO_PWM_MAX) cmd = SERVO_PWM_MAX;
  if (cmd > cur + GIMBAL_SLEW) cmd = cur + GIMBAL_SLEW;
  if (cmd < cur - GIMBAL_SLEW) cmd = cur - GIMBAL_SLEW;
  return cmd;
}

/* 向 ESP8266 发一串 AT 字符（USART6 阻塞发送） */
static void esp8266_send(const char *s)
{
  HAL_UART_Transmit(&huart6, (const uint8_t *)s, strlen(s), 500);
}

/* 等 ESP8266 回复里出现 expect 字样：出现返回 1，超时返回 0 */
static int esp8266_wait(const char *expect, uint32_t timeout_ms)
{
  char buf[96];
  uint16_t len = 0;
  uint32_t start = HAL_GetTick();
  memset(buf, 0, sizeof(buf));
  while (HAL_GetTick() - start < timeout_ms)
  {
    uint8_t ch;
    if (HAL_UART_Receive(&huart6, &ch, 1, 20) == HAL_OK)
    {
      if (len == sizeof(buf) - 1)         /* 缓冲满了丢前一半，保留最新尾巴 */
      {
        memmove(buf, buf + 48, len - 48);
        len -= 48;
      }
      buf[len++] = (char)ch;
      if (strstr(buf, expect) != NULL) return 1;
    }
  }
  return 0;
}
/* USER CODE END Application */

