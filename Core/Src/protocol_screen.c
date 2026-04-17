#include "protocol_screen.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static ScreenRxData_t g_screen;
static char g_line[96];
static uint8_t g_idx = 0U;

extern volatile uint32_t g_sys_ms;

static uint8_t ParseU8List(const char *s, uint8_t *out, uint8_t max_cnt)
{
    uint8_t cnt = 0U;
    char *endptr;
    unsigned long v;

    if (s == NULL || out == NULL || max_cnt == 0U)
    {
        return 0U;
    }

    while (*s != '\0' && cnt < max_cnt)
    {
        while (*s == ' ' || *s == ',')
        {
            s++;
        }

        if (*s == '\0')
        {
            break;
        }

        v = strtoul(s, &endptr, 10);
        if (endptr == s)
        {
            break;
        }

        out[cnt++] = (uint8_t)v;
        s = endptr;

        while (*s == ' ')
        {
            s++;
        }

        if (*s == ',')
        {
            s++;
        }
    }

    return cnt;
}

static void ProtocolScreen_ParseLine(char *line)
{
    unsigned int t = 0U;
    unsigned int rid = 0U;
    float x = 0.0f, y = 0.0f;
    uint8_t temp_route[USER_ROUTE_MAX_LEN];
    uint8_t len = 0U;

    if (strncmp(line, "START", 5) == 0)
    {
        g_screen.start_cmd = 1U;
        g_screen.update_ms = g_sys_ms;
    }
    else if (strncmp(line, "STOP", 4) == 0)
    {
        g_screen.stop_cmd = 1U;
        g_screen.update_ms = g_sys_ms;
    }
    else if (strncmp(line, "ZEROIMU", 7) == 0)
    {
        g_screen.imu_zero_cmd = 1U;
        g_screen.update_ms = g_sys_ms;
    }
    else if (sscanf(line, "TASK=%u", &t) == 1)
    {
        g_screen.task_id = (uint8_t)t;
        g_screen.update_ms = g_sys_ms;
    }
    else if (sscanf(line, "POINT=%f,%f", &x, &y) == 2)
    {
        g_screen.point_x_mm = x;
        g_screen.point_y_mm = y;
        g_screen.point_valid = 1U;
        g_screen.update_ms = g_sys_ms;
    }
    else if (sscanf(line, "REGION=%u", &rid) == 1)
    {
        g_screen.region_id = (uint8_t)rid;
        g_screen.region_valid = 1U;
        g_screen.update_ms = g_sys_ms;
    }
    else if (strncmp(line, "ROUTE=", 6) == 0)
    {
        memset(temp_route, 0, sizeof(temp_route));
        len = ParseU8List(line + 6, temp_route, USER_ROUTE_MAX_LEN);
        if (len > 0U)
        {
            memcpy(g_screen.route, temp_route, len);
            g_screen.route_len = len;
            g_screen.route_pass_mode = 0U;
            g_screen.route_valid = 1U;
            g_screen.update_ms = g_sys_ms;
        }
    }
    else if (strncmp(line, "PASS=", 5) == 0)
    {
        memset(temp_route, 0, sizeof(temp_route));
        len = ParseU8List(line + 5, temp_route, USER_ROUTE_MAX_LEN);
        if (len > 0U)
        {
            memcpy(g_screen.route, temp_route, len);
            g_screen.route_len = len;
            g_screen.route_pass_mode = 1U;
            g_screen.route_valid = 1U;
            g_screen.update_ms = g_sys_ms;
        }
    }
    else if (strncmp(line, "ROUND=", 6) == 0)
    {
        unsigned int a = 0U, b = 0U;
        if (sscanf(line, "ROUND=%u,%u", &a, &b) == 2)
        {
            g_screen.roundtrip_a = (uint8_t)a;
            g_screen.roundtrip_b = (uint8_t)b;
            g_screen.roundtrip_valid = 1U;
            g_screen.update_ms = g_sys_ms;
        }
    }
}

void ProtocolScreen_Init(void)
{
    memset(&g_screen, 0, sizeof(g_screen));
    memset(g_line, 0, sizeof(g_line));
    g_idx = 0U;
}

ScreenRxData_t* ProtocolScreen_GetData(void)
{
    return &g_screen;
}

void ProtocolScreen_RxByte(uint8_t byte)
{
    if (byte == '\r')
    {
        return;
    }

    if (byte == '\n')
    {
        g_line[g_idx] = '\0';
        ProtocolScreen_ParseLine(g_line);
        g_idx = 0U;
        return;
    }

    if (g_idx < (sizeof(g_line) - 1U))
    {
        g_line[g_idx++] = (char)byte;
    }
    else
    {
        g_idx = 0U;
    }
}

void ProtocolScreen_SendText(const char *text)
{
    if (text == NULL)
    {
        return;
    }

    HAL_UART_Transmit(&huart6, (uint8_t *)text, (uint16_t)strlen(text), 50U);
}
