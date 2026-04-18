#ifndef __VISION_FRAME_H__
#define __VISION_FRAME_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* USB 视觉协议保持为:
   0xAA + current_x(2B) + current_y(2B) + 0xA5
   当前版本只接收“当前坐标”，目标点完全由下位机状态机自行决定。 */
#define VISION_FRAME_HEAD                 0xAAU
#define VISION_FRAME_TAIL                 0xA5U
#define VISION_FRAME_PAYLOAD_SIZE            4U

typedef struct
{
  uint16_t current_x;
  uint16_t current_y;
  uint16_t target_x;      /* 当前版本保留字段，固定清零，便于兼容旧代码。 */
  uint16_t target_y;      /* 当前版本保留字段，固定清零，便于兼容旧代码。 */
  uint8_t frame_valid;
  uint32_t recv_tick_ms;
} VisionFrame_TypeDef;

typedef struct
{
  uint8_t payload[VISION_FRAME_PAYLOAD_SIZE];
  uint8_t payload_index;
  uint8_t receiving;
  VisionFrame_TypeDef latest_frame;
} VisionFrameParser_TypeDef;

/**
  * @brief  初始化视觉帧解析器对象。
  * @param  p_parser: 视觉帧解析器对象指针。
  * @retval None
  */
void VisionFrameParser_Init(VisionFrameParser_TypeDef *p_parser);

/**
  * @brief  轮询 USB 接收缓冲区，并尝试解析出新的视觉帧。
  * @param  p_parser: 视觉帧解析器对象指针。
  * @param  now_ms: 当前系统毫秒节拍。
  * @retval 1 表示至少解析出 1 帧新数据，0 表示本次没有新帧。
  */
uint8_t VisionFrameParser_PollUsb(VisionFrameParser_TypeDef *p_parser, uint32_t now_ms);

/**
  * @brief  获取最近一次解析成功的视觉帧。
  * @param  p_parser: 视觉帧解析器对象指针。
  * @retval 指向最近视觉帧的常量指针；若参数无效则返回 NULL。
  */
const VisionFrame_TypeDef *VisionFrameParser_GetLatest(const VisionFrameParser_TypeDef *p_parser);

#ifdef __cplusplus
}
#endif

#endif /* __VISION_FRAME_H__ */

