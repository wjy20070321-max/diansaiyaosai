#ifndef __PLATE_TASK_FSM_H__
#define __PLATE_TASK_FSM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 9 个目标点与输入路径缓存长度宏定义。
   后续如果题目需要更多中间点，只需要把缓存长度调大即可。 */
#define PLATE_TASK_TARGET_POINT_COUNT         9U
#define PLATE_TASK_MAX_INPUT_POINT_COUNT     16U

/* 到达判定和各任务停留时间宏定义，便于后续现场调参。
   这里保留一套通用半径，同时给发挥题单独留一套更严格的半径。
   这样基础题保持现在的容忍度，发挥题则要求更接近目标点才开始/继续停留计时。 */
#define PLATE_TASK_DEFAULT_ARRIVE_RADIUS_PX     28U
#define PLATE_TASK_DEFAULT_STAY_RADIUS_PX       36U
#define PLATE_TASK_ADVANCED_ARRIVE_RADIUS_PX   24U
#define PLATE_TASK_ADVANCED_STAY_RADIUS_PX     28U
#define PLATE_TASK_ADVANCED_STAY_MS           3000U
#define PLATE_TASK_ADVANCED_LOOP_STAY_MS      4000U
#define PLATE_TASK_ADVANCED4_FINAL_STAY_MS    3000U

/* 九个目标点像素坐标宏定义。
   为了调参阶段尽量不要贴近画面边框，这里先定义一套“靠内”的安全基准网格；
   然后在安全网格的基础上叠加前面逐点测出来的补偿量。
   你后续如果还想整体往里收或往外放，只需要先改这 6 个安全基准坐标。 */
#define PLATE_TASK_SAFE_LEFT_X      40U
#define PLATE_TASK_SAFE_CENTER_X   320U
#define PLATE_TASK_SAFE_RIGHT_X    600U
#define PLATE_TASK_SAFE_TOP_Y       40U
#define PLATE_TASK_SAFE_CENTER_Y   320U
#define PLATE_TASK_SAFE_BOTTOM_Y   600U

#define PLATE_TASK_POINT1_X   (PLATE_TASK_SAFE_LEFT_X)
#define PLATE_TASK_POINT1_Y   (PLATE_TASK_SAFE_TOP_Y)

#define PLATE_TASK_POINT2_X   (PLATE_TASK_SAFE_CENTER_X)
#define PLATE_TASK_POINT2_Y   (PLATE_TASK_SAFE_TOP_Y)

#define PLATE_TASK_POINT3_X   (PLATE_TASK_SAFE_RIGHT_X)
#define PLATE_TASK_POINT3_Y   (PLATE_TASK_SAFE_TOP_Y)

#define PLATE_TASK_POINT4_X   (PLATE_TASK_SAFE_LEFT_X)
#define PLATE_TASK_POINT4_Y   (PLATE_TASK_SAFE_CENTER_Y)

#define PLATE_TASK_POINT5_X   (PLATE_TASK_SAFE_CENTER_X)
#define PLATE_TASK_POINT5_Y   (PLATE_TASK_SAFE_CENTER_Y)

#define PLATE_TASK_POINT6_X   (PLATE_TASK_SAFE_RIGHT_X)
#define PLATE_TASK_POINT6_Y   (PLATE_TASK_SAFE_CENTER_Y)

#define PLATE_TASK_POINT7_X   (PLATE_TASK_SAFE_LEFT_X)
#define PLATE_TASK_POINT7_Y   (PLATE_TASK_SAFE_BOTTOM_Y)

#define PLATE_TASK_POINT8_X   (PLATE_TASK_SAFE_CENTER_X)
#define PLATE_TASK_POINT8_Y   (PLATE_TASK_SAFE_BOTTOM_Y)

#define PLATE_TASK_POINT9_X   (PLATE_TASK_SAFE_RIGHT_X)
#define PLATE_TASK_POINT9_Y   (PLATE_TASK_SAFE_BOTTOM_Y)

/* 默认调平参考点宏定义。
   上电后与后续复位调平，都回到这个参考点对应的默认平衡状态。 */
#define PLATE_TASK_LEVEL_DEFAULT_TARGET_X   PLATE_TASK_POINT5_X
#define PLATE_TASK_LEVEL_DEFAULT_TARGET_Y   PLATE_TASK_POINT5_Y

typedef enum
{
  PLATE_TASK_MODE_LEVEL = 0U,      /* 默认调平/复位模式，舵机回到默认平衡状态。 */
  PLATE_TASK_MODE_BASIC = 1U,      /* 基础题模式。 */
  PLATE_TASK_MODE_ADVANCED = 2U    /* 发挥题模式。 */
} PlateTask_ModeTypeDef;

typedef enum
{
  PLATE_TASK_ID_NONE = 0U,
  PLATE_TASK_ID_BASIC1 = 1U,
  PLATE_TASK_ID_BASIC2 = 2U,
  PLATE_TASK_ID_ADVANCED1 = 3U,
  PLATE_TASK_ID_ADVANCED2 = 4U,
  PLATE_TASK_ID_ADVANCED3 = 5U,
  PLATE_TASK_ID_ADVANCED4 = 6U
} PlateTask_TaskIdTypeDef;

typedef enum
{
  PLATE_TASK_RUN_IDLE = 0U,        /* 当前无任务，或处于默认调平状态。 */
  PLATE_TASK_RUN_WAIT_INPUT,       /* 已选题，正在等待 AA...BB 输入目标点。 */
  PLATE_TASK_RUN_MOVING,           /* 正在向当前目标点运动。 */
  PLATE_TASK_RUN_STAYING,          /* 已进入目标区域，正在做停留计时。 */
  PLATE_TASK_RUN_HOLDING           /* 任务已完成，继续稳定保持在最后目标点。 */
} PlateTask_RunStateTypeDef;

typedef struct
{
  uint16_t x;
  uint16_t y;
} PlateTask_PointTypeDef;

typedef struct
{
  uint16_t arrive_radius_px;
  uint16_t stay_radius_px;
  uint16_t level_target_x;
  uint16_t level_target_y;
  uint16_t advanced_stay_ms;
  uint16_t advanced_loop_stay_ms;
  uint16_t advanced4_final_stay_ms;
  PlateTask_PointTypeDef target_points[PLATE_TASK_TARGET_POINT_COUNT + 1U];
} PlateTask_ConfigTypeDef;

typedef struct
{
  PlateTask_ModeTypeDef mode;
  PlateTask_TaskIdTypeDef task_id;
  PlateTask_RunStateTypeDef run_state;
  uint8_t current_target_index;
  uint8_t current_target_id;
  uint8_t target_count;
  uint8_t input_active;
  uint8_t task_finished;
  uint8_t task_timeout;
  uint8_t last_command;
  uint16_t active_target_x;
  uint16_t active_target_y;
  uint32_t task_start_tick_ms;
  uint32_t stay_start_tick_ms;
} PlateTask_StateTypeDef;

/**
  * @brief  获取题目状态机配置表，便于外部直接修改九点像素坐标和判定参数。
  * @param  None
  * @retval 返回全局状态机配置结构体指针。
  */
PlateTask_ConfigTypeDef *PlateTaskFsm_GetConfig(void);

/**
  * @brief  初始化题目状态机、九点目标表和默认调平参考点。
  * @param  level_target_x: 默认调平参考点 X 坐标。
  * @param  level_target_y: 默认调平参考点 Y 坐标。
  * @retval None
  */
void PlateTaskFsm_Init(uint16_t level_target_x, uint16_t level_target_y);

/**
  * @brief  按串口屏单字节协议处理 1 个输入字节。
  * @param  command: 串口屏发送的单字节命令。
  * @retval 1 表示该字节已被状态机处理，0 表示当前字节不属于状态机协议。
  */
uint8_t PlateTaskFsm_HandleCommand(uint8_t command);

/**
  * @brief  按当前任务模式更新目标点选择和题目推进状态。
  * @param  now_ms: 当前系统毫秒节拍。
  * @param  current_x: 当前视觉检测到的 X 坐标。
  * @param  current_y: 当前视觉检测到的 Y 坐标。
  * @param  external_target_x: 保留参数，当前版本未使用。
  * @param  external_target_y: 保留参数，当前版本未使用。
  * @param  vision_valid: 当前视觉数据是否有效，1 表示有效，0 表示无效。
  * @retval None
  */
void PlateTaskFsm_Update(uint32_t now_ms,
                         uint16_t current_x,
                         uint16_t current_y,
                         uint16_t external_target_x,
                         uint16_t external_target_y,
                         uint8_t vision_valid);

/**
  * @brief  判断当前模式是否允许控制层继续跑闭环控制。
  * @param  None
  * @retval 1 表示允许控制，0 表示应回到默认平衡状态。
  */
uint8_t PlateTaskFsm_IsControlEnabled(void);

/**
  * @brief  获取当前题目状态机运行状态。
  * @param  None
  * @retval 返回状态机只读状态指针。
  */
const PlateTask_StateTypeDef *PlateTaskFsm_GetState(void);

/**
  * @brief  主动请求进入默认调平模式，并清除已接收的目标点缓存。
  * @param  None
  * @retval None
  */
void PlateTaskFsm_RequestLevelReset(void);

#ifdef __cplusplus
}
#endif

#endif /* __PLATE_TASK_FSM_H__ */



