#include "vision_control.h"

#include <string.h>

#include "control_tick_device.h"
#include "gyro_debug_servo.h"
#include "jy61p.h"
#include "my_usb_device.h"
#include "serial_screen_device.h"
#include "system_time_device.h"

/* 默认平衡与正式控制参数宏定义。
   其中 X/Y 两路舵机的 balance_us 就是平台默认平衡角度对应的中值脉宽，
   后续现场调平时你优先改这两个宏即可。 */
#define VISION_CONTROL_DEFAULT_LEVEL_TARGET_X   PLATE_TASK_LEVEL_DEFAULT_TARGET_X
#define VISION_CONTROL_DEFAULT_LEVEL_TARGET_Y   PLATE_TASK_LEVEL_DEFAULT_TARGET_Y
#define VISION_CONTROL_FRAME_TIMEOUT_MS         200U
#define VISION_CONTROL_FRAME_HOLD_MS            400U
#define VISION_CONTROL_FRAME_HOLD_ERROR_PX      20.0f
#define VISION_CONTROL_FRAME_HOLD_OUTPUT_US     40.0f
#define VISION_CONTROL_FIXED_DT_S               0.005f
/* 视觉分辨率当前为 640x640。
   这里把位置环/速度环死区单独提成宏，便于后续现场快速微调。 */
#define VISION_CONTROL_POSITION_DEADBAND_PX         3.0f
#define VISION_CONTROL_VELOCITY_DEADBAND_PXPS       3.0f
#define VISION_CONTROL_POSITION_OUTPUT_LIMIT_PXPS   160.0f
#define VISION_CONTROL_X_POSITION_KP                1.50f
#define VISION_CONTROL_Y_POSITION_KP                1.20f
#define VISION_CONTROL_X_POSITION_KI                0.14f
#define VISION_CONTROL_Y_POSITION_KI                0.03f
#define VISION_CONTROL_POSITION_INTEGRAL_LIMIT      250.0f
/* 舵机角度与脉宽采用 0°=500us、180°=2500us 线性映射。
   40° 对应的脉宽偏差约为 444.4us。
   这里把正式控制的最大偏转统一限制为“相对中心 ±40°”。 */
#define VISION_CONTROL_SERVO_DELTA_LIMIT_US     444.4f
/* 速度环先用更小的软限幅调试，避免速度误差较大时持续打到机械硬限幅。 */
#define VISION_CONTROL_VELOCITY_OUTPUT_LIMIT_US 320.0f

#define VISION_CONTROL_X_SERVO_BALANCE_US      1362.8f    /* X 轴零偏按轻微偏左现象做收尾微调 */
#define VISION_CONTROL_X_SERVO_MIN_US           944.4f    /* 约 40° = 80° - 40° */
#define VISION_CONTROL_X_SERVO_MAX_US          1833.3f    /* 约 120° = 80° + 40° */

#define VISION_CONTROL_Y_SERVO_BALANCE_US      1500.0f    /* 90° */
#define VISION_CONTROL_Y_SERVO_MIN_US          1055.6f    /* 约 50° = 90° - 40° */
#define VISION_CONTROL_Y_SERVO_MAX_US          1944.4f    /* 约 130° = 90° + 40° */

/* 顶层控制器负责把各模块串起来:
   1. USB 视觉收帧，只接收当前坐标
   2. USART1 串口屏命令与页面显示
   3. UART4 陀螺仪调试舵机
   4. TIM11 固定 5ms 控制节拍
   5. TIM2 双轴舵机输出
   6. TIM12 双路调试舵机输出 */
typedef struct
{
  VisionControl_ConfigTypeDef config;
  VisionControl_StateTypeDef state;
  VisionFrameParser_TypeDef parser;
  VisionAxis_TypeDef x_axis;
  VisionAxis_TypeDef y_axis;
  uint32_t last_frame_tick_ms;
  volatile uint8_t control_tick_pending;
  uint8_t last_screen_target_id;
} VisionControl_ContextTypeDef;

static VisionControl_ConfigTypeDef g_vision_control_config =
{
  .center_x = VISION_CONTROL_DEFAULT_LEVEL_TARGET_X,
  .center_y = VISION_CONTROL_DEFAULT_LEVEL_TARGET_Y,
  .frame_timeout_ms = VISION_CONTROL_FRAME_TIMEOUT_MS,
  .x_axis =
  {
    .servo_tim = SERVO_TIM2_1,
    .servo_center_us = VISION_CONTROL_X_SERVO_BALANCE_US,
    .servo_min_us = VISION_CONTROL_X_SERVO_MIN_US,
    .servo_max_us = VISION_CONTROL_X_SERVO_MAX_US,
    .servo_direction = -1.0f,
    .velocity_filter_alpha = 0.2f,
    .position_pid =
    {
      .kp = VISION_CONTROL_X_POSITION_KP,
      .ki = VISION_CONTROL_X_POSITION_KI,
      .kd = 0.00f,
      .integral_limit = VISION_CONTROL_POSITION_INTEGRAL_LIMIT,
      .output_limit = VISION_CONTROL_POSITION_OUTPUT_LIMIT_PXPS,
      .deadband = VISION_CONTROL_POSITION_DEADBAND_PX
    },
    .velocity_pid =
    {
      .kp = 1.8f,
      .ki = 0.00f,
      .kd = 0.00f,
      .integral_limit = 0.0f,
      .output_limit = VISION_CONTROL_VELOCITY_OUTPUT_LIMIT_US,
      .deadband = VISION_CONTROL_VELOCITY_DEADBAND_PXPS
    }
  },
  .y_axis =
  {
    .servo_tim = SERVO_TIM2_2,
    .servo_center_us = VISION_CONTROL_Y_SERVO_BALANCE_US,
    .servo_min_us = VISION_CONTROL_Y_SERVO_MIN_US,
    .servo_max_us = VISION_CONTROL_Y_SERVO_MAX_US,
    .servo_direction = 1.0f,
    .velocity_filter_alpha = 0.2f,
    .position_pid =
    {
      .kp = VISION_CONTROL_Y_POSITION_KP,
      .ki = VISION_CONTROL_Y_POSITION_KI,
      .kd = 0.00f,
      .integral_limit = VISION_CONTROL_POSITION_INTEGRAL_LIMIT,
      .output_limit = VISION_CONTROL_POSITION_OUTPUT_LIMIT_PXPS,
      .deadband = VISION_CONTROL_POSITION_DEADBAND_PX
    },
    .velocity_pid =
    {
      .kp = 2.0f,
      .ki = 0.00f,
      .kd = 0.00f,
      .integral_limit = 0.0f,
      .output_limit = VISION_CONTROL_VELOCITY_OUTPUT_LIMIT_US,
      .deadband = VISION_CONTROL_VELOCITY_DEADBAND_PXPS
    }
  }
};

static VisionControl_ContextTypeDef g_vision_control;

/**
  * @brief  简单绝对值函数，避免为了几处比较额外引入标准库浮点接口。
  * @param  value: 输入浮点值。
  * @retval 返回 value 的绝对值。
  */
static float VisionControl_Abs(float value)
{
  if (value < 0.0f)
  {
    return -value;
  }

  return value;
}

/**
  * @brief  把 X/Y 双轴内部状态同步到对外状态结构体。
  * @param  None
  * @retval None
  */
static void VisionControl_SyncAxisState(void)
{
  const VisionAxis_StateTypeDef *p_x_state;
  const VisionAxis_StateTypeDef *p_y_state;

  p_x_state = VisionAxis_GetState(&g_vision_control.x_axis);
  p_y_state = VisionAxis_GetState(&g_vision_control.y_axis);

  if (p_x_state != (const VisionAxis_StateTypeDef *)0)
  {
    g_vision_control.state.x_axis = *p_x_state;
  }

  if (p_y_state != (const VisionAxis_StateTypeDef *)0)
  {
    g_vision_control.state.y_axis = *p_y_state;
  }
}

/**
  * @brief  向串口屏实时刷新当前目标点编号 n2。
  * @param  target_id: 当前正在追踪的目标点编号，范围 0~9。
  * @retval None
  */
static void VisionControl_UpdateScreenTargetPoint(uint8_t target_id)
{
  if (g_vision_control.last_screen_target_id == target_id)
  {
    return;
  }

  g_vision_control.last_screen_target_id = target_id;
  (void)SerialScreen_Device_SendNumberValue("n2", (uint32_t)target_id);
}

/**
  * @brief  统一执行双轴回中并清空控制器内部状态。
  * @param  None
  * @retval None
  */
static void VisionControl_ResetOutput(void)
{
  VisionAxis_Reset(&g_vision_control.x_axis);
  VisionAxis_Reset(&g_vision_control.y_axis);
  VisionControl_SyncAxisState();
  g_vision_control.state.control_enabled = 0U;
}

/**
  * @brief  读取陀螺仪当前 roll/pitch，并更新 TIM12 双路调试舵机。
  * @param  None
  * @retval None
  */
void VisionControl_UpdateGyroDebugServo(void)
{
  float roll = 0.0f;
  float pitch = 0.0f;

  JY61P_GetEuler(&roll, &pitch, (float *)0);
  GyroDebugServo_Update(-roll, pitch);
}

/**
  * @brief  把最新视觉帧中的当前坐标实时刷新到串口屏的 n0/n1 数值控件。
  * @param  p_frame: 最近一次解析成功的视觉帧指针。
  * @retval None
  */
static void VisionControl_UpdateScreenCurrentCoordinate(const VisionFrame_TypeDef *p_frame)
{
  if ((p_frame == (const VisionFrame_TypeDef *)0) || (p_frame->frame_valid == 0U))
  {
    return;
  }

  (void)SerialScreen_Device_SendNumberValue("n0", (uint32_t)p_frame->current_x);
  (void)SerialScreen_Device_SendNumberValue("n1", (uint32_t)p_frame->current_y);
}

/**
  * @brief  用最新视觉帧更新双轴的当前位置和速度估计。
  * @param  None
  * @retval None
  */
static void VisionControl_UpdateFrameMeasurement(void)
{
  const VisionFrame_TypeDef *p_frame;
  float frame_dt_s = 0.0f;

  p_frame = VisionFrameParser_GetLatest(&g_vision_control.parser);
  if ((p_frame == (const VisionFrame_TypeDef *)0) || (p_frame->frame_valid == 0U))
  {
    return;
  }

  g_vision_control.state.frame = *p_frame;
  VisionControl_UpdateScreenCurrentCoordinate(p_frame);

  /* 视觉帧到达时间不固定，因此速度估计必须使用真实帧间隔。 */
  if ((g_vision_control.last_frame_tick_ms != 0U) &&
      (p_frame->recv_tick_ms > g_vision_control.last_frame_tick_ms))
  {
    frame_dt_s =
      ((float)(p_frame->recv_tick_ms - g_vision_control.last_frame_tick_ms)) * 0.001f;
  }

  VisionAxis_UpdateMeasurement(&g_vision_control.x_axis, (float)p_frame->current_x, frame_dt_s);
  VisionAxis_UpdateMeasurement(&g_vision_control.y_axis, (float)p_frame->current_y, frame_dt_s);

  g_vision_control.last_frame_tick_ms = p_frame->recv_tick_ms;
  VisionControl_SyncAxisState();
}

/**
  * @brief  消费串口屏输入字节，并交给题目状态机处理。
  * @param  None
  * @retval None
  */
static void VisionControl_ServiceScreenCommand(void)
{
  uint8_t command;
  uint8_t handled = 0U;

  while (SerialScreen_Device_PopCommand(&command) != 0U)
  {
    /* 0xCC 为最高优先级复位调平命令。
       串口屏一旦发出该字节，立即退出当前所有题目流程，
       清空已缓存的输入序列，并让 TIM2 双轴回到默认平衡状态。 */
    if (command == SERIAL_SCREEN_CMD_RESET_LEVEL)
    {
      VisionControl_RequestLevelReset();
      handled = 1U;
      continue;
    }

    if (PlateTaskFsm_HandleCommand(command) != 0U)
    {
      handled = 1U;
    }
  }

  if (handled != 0U)
  {
    /* 只要任务协议有变化，就清一次积分与输出，避免切题瞬间冲击平台。 */
    VisionControl_ResetOutput();
  }
}

/**
  * @brief  取出 1 个待执行的 TIM11 控制节拍。
  * @param  None
  * @retval 1 表示成功取到 1 个待执行节拍，0 表示当前没有待执行节拍。
  */
static uint8_t VisionControl_TakeTick(void)
{
  uint8_t has_tick = 0U;

  if (g_vision_control.control_tick_pending > 0U)
  {
    g_vision_control.control_tick_pending--;
    g_vision_control.state.control_tick_pending = g_vision_control.control_tick_pending;
    has_tick = 1U;
  }

  return has_tick;
}

/**
  * @brief  按固定 5ms 周期执行 1 次完整控制拍。
  * @param  now_ms: 当前系统毫秒节拍。
  * @retval None
  */
static void VisionControl_RunFixedTick(uint32_t now_ms)
{
  const PlateTask_StateTypeDef *p_task_state;
  uint8_t frame_is_fresh = 0U;
  uint32_t frame_age_ms = 0xFFFFFFFFU;
  float position_error_x = 0.0f;
  float position_error_y = 0.0f;

  if ((g_vision_control.state.frame.frame_valid != 0U) &&
      (now_ms >= g_vision_control.state.frame.recv_tick_ms))
  {
    frame_age_ms = now_ms - g_vision_control.state.frame.recv_tick_ms;

    if (frame_age_ms <= g_vision_control.config.frame_timeout_ms)
    {
      frame_is_fresh = 1U;
    }
  }

  g_vision_control.state.vision_online = frame_is_fresh;

  PlateTaskFsm_Update(now_ms,
                      g_vision_control.state.frame.current_x,
                      g_vision_control.state.frame.current_y,
                      0U,
                      0U,
                      frame_is_fresh);

  p_task_state = PlateTaskFsm_GetState();
  g_vision_control.state.mode = p_task_state->mode;
  g_vision_control.state.task_id = (uint8_t)p_task_state->task_id;
  g_vision_control.state.task_state = p_task_state->run_state;
  g_vision_control.state.task_finished = p_task_state->task_finished;
  g_vision_control.state.task_timeout = p_task_state->task_timeout;
  g_vision_control.state.active_target_x = p_task_state->active_target_x;
  g_vision_control.state.active_target_y = p_task_state->active_target_y;
  g_vision_control.state.target_point_id = p_task_state->current_target_id;

  position_error_x =
    (float)((int32_t)g_vision_control.state.active_target_x -
            (int32_t)g_vision_control.state.frame.current_x);
  position_error_y =
    (float)((int32_t)g_vision_control.state.active_target_y -
            (int32_t)g_vision_control.state.frame.current_y);

  VisionControl_UpdateScreenTargetPoint(p_task_state->current_target_id);

  if (PlateTaskFsm_IsControlEnabled() == 0U)
  {
    VisionControl_ResetOutput();
    return;
  }

  if (g_vision_control.state.frame.frame_valid == 0U)
  {
    VisionControl_ResetOutput();
    return;
  }

  if (frame_is_fresh == 0U)
  {
    /* 视觉偶发丢 1~2 帧时，保持上一拍舵机输出，
       避免平台因为瞬时回中把小球又放跑。
       但只有“已经接近目标，且当前控制输出不大”时才允许保持；
       如果此时离目标还远、舵机还在大幅动作，就不能盲目沿用旧输出，
       否则小球可能会被继续推离目标。 */
    if ((frame_age_ms <= VISION_CONTROL_FRAME_HOLD_MS) &&
        (VisionControl_Abs(position_error_x) <= VISION_CONTROL_FRAME_HOLD_ERROR_PX) &&
        (VisionControl_Abs(position_error_y) <= VISION_CONTROL_FRAME_HOLD_ERROR_PX) &&
        (VisionControl_Abs(g_vision_control.state.x_axis.servo_output_us) <=
         VISION_CONTROL_FRAME_HOLD_OUTPUT_US) &&
        (VisionControl_Abs(g_vision_control.state.y_axis.servo_output_us) <=
         VISION_CONTROL_FRAME_HOLD_OUTPUT_US))
    {
      g_vision_control.state.control_enabled = 1U;
      return;
    }

    VisionControl_ResetOutput();
    return;
  }

  if ((VisionAxis_Run(&g_vision_control.x_axis,
                      (float)g_vision_control.state.active_target_x,
                      (float)g_vision_control.state.frame.current_x,
                      VISION_CONTROL_FIXED_DT_S) != BSP_OK) ||
      (VisionAxis_Run(&g_vision_control.y_axis,
                      (float)g_vision_control.state.active_target_y,
                      (float)g_vision_control.state.frame.current_y,
                      VISION_CONTROL_FIXED_DT_S) != BSP_OK))
  {
    VisionControl_ResetOutput();
    return;
  }

  g_vision_control.state.control_enabled = 1U;
  VisionControl_SyncAxisState();
}

/**
  * @brief  获取顶层控制器配置结构体，便于外部调参。
  * @param  None
  * @retval 指向全局控制配置的指针。
  */
VisionControl_ConfigTypeDef *VisionControl_GetConfig(void)
{
  return &g_vision_control_config;
}

/**
  * @brief  初始化顶层控制器及其依赖模块。
  * @param  None
  * @retval BSP_OK 表示初始化成功，BSP_ERROR 表示其中某个子模块初始化失败。
  */
BSP_StatusTypeDef VisionControl_Init(void)
{
  memset(&g_vision_control, 0, sizeof(g_vision_control));

  g_vision_control.config = g_vision_control_config;
  g_vision_control.state.active_target_x = g_vision_control.config.center_x;
  g_vision_control.state.active_target_y = g_vision_control.config.center_y;
  g_vision_control.last_screen_target_id = 0xFFU;

  Usb_Device_ClearRxOverflow();
  Usb_Device_ClearRxBuffer();
  VisionFrameParser_Init(&g_vision_control.parser);

  if (VisionAxis_Init(&g_vision_control.x_axis, &g_vision_control.config.x_axis) != BSP_OK)
  {
    return BSP_ERROR;
  }

  if (VisionAxis_Init(&g_vision_control.y_axis, &g_vision_control.config.y_axis) != BSP_OK)
  {
    return BSP_ERROR;
  }

  if (SerialScreen_Device_Init() != BSP_OK)
  {
    return BSP_ERROR;
  }

  if (JY61P_Init() != BSP_OK)
  {
    return BSP_ERROR;
  }

  if (GyroDebugServo_Init() != BSP_OK)
  {
    return BSP_ERROR;
  }

  PlateTaskFsm_Init(g_vision_control.config.center_x, g_vision_control.config.center_y);
  VisionControl_ResetOutput();
  VisionControl_UpdateScreenTargetPoint(0U);

  if (ControlTick_Device_Init(VisionControl_OnControlTick) != BSP_OK)
  {
    return BSP_ERROR;
  }

  return BSP_OK;
}

/**
  * @brief  顶层主循环任务，负责消费输入并执行待处理的控制拍。
  * @param  None
  * @retval None
  */
void VisionControl_Task(void)
{
  uint32_t now_ms;

  now_ms = SystemTime_Device_GetTickMs();

  if (VisionFrameParser_PollUsb(&g_vision_control.parser, now_ms) != 0U)
  {
    VisionControl_UpdateFrameMeasurement();
  }

  //
  VisionControl_ServiceScreenCommand();

  while (VisionControl_TakeTick() != 0U)
  {
    now_ms = SystemTime_Device_GetTickMs();
    VisionControl_RunFixedTick(now_ms);
    g_vision_control.state.control_tick_count++;
  }

  if (PlateTaskFsm_GetState()->mode == PLATE_TASK_MODE_LEVEL)
  {
    VisionControl_UpdateGyroDebugServo();
  }
}

/**
  * @brief  主动请求进入默认调平模式，并清空串口屏已缓存的任务输入。
  * @param  None
  * @retval None
  */
void VisionControl_RequestLevelReset(void)
{
  SerialScreen_Device_ClearCommandQueue();
  PlateTaskFsm_RequestLevelReset();
  VisionControl_ResetOutput();
  VisionControl_UpdateScreenTargetPoint(0U);
}

/**
  * @brief  更新控制层中心参考点。
  * @param  center_x: 新的中心参考点 X 坐标。
  * @param  center_y: 新的中心参考点 Y 坐标。
  * @retval None
  */
void VisionControl_SetCenter(uint16_t center_x, uint16_t center_y)
{
  PlateTask_ConfigTypeDef *p_task_config;

  g_vision_control_config.center_x = center_x;
  g_vision_control_config.center_y = center_y;

  g_vision_control.config.center_x = center_x;
  g_vision_control.config.center_y = center_y;

  p_task_config = PlateTaskFsm_GetConfig();
  p_task_config->level_target_x = center_x;
  p_task_config->level_target_y = center_y;
}

/**
  * @brief  获取当前控制器运行状态。
  * @param  None
  * @retval 指向控制器状态结构体的常量指针。
  */
const VisionControl_StateTypeDef *VisionControl_GetState(void)
{
  return &g_vision_control.state;
}

/**
  * @brief  由 Devices 层调用，通知控制层新增 1 个待执行控制拍。
  * @param  None
  * @retval None
  */
void VisionControl_OnControlTick(void)
{
  if (g_vision_control.control_tick_pending < 10U)
  {
    g_vision_control.control_tick_pending++;
  }

  g_vision_control.state.control_tick_pending = g_vision_control.control_tick_pending;
}
