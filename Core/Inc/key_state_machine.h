/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    key_state_machine.h
  * @brief   ����״̬��ͷ�ļ�
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __KEY_STATE_MACHINE_H
#define __KEY_STATE_MACHINE_H

#include "stm32h7xx_hal.h"

// �������Ŷ��壬�����ʵ��Ӳ���޸�
#define K1_GPIO_Port GPIOE
#define K1_Pin GPIO_PIN_2
#define K2_GPIO_Port GPIOE
#define K2_Pin GPIO_PIN_3
#define K3_GPIO_Port GPIOE
#define K3_Pin GPIO_PIN_4
#define K4_GPIO_Port GPIOE
#define K4_Pin GPIO_PIN_5


// ������������
#define KEY_LONG_PRESS_MS 1000    // �����ж�ʱ��(ms)
#define KEY_DOUBLE_CLICK_MS 300   // ˫���ж�ʱ����(ms)
#define KEY_DEBOUNCE_MS 20        // ��������ʱ�䣨���˻�е������Լ10-20ms��

// ����״̬ö��
typedef enum {
    KEY_STATE_IDLE,         // ����״̬
    KEY_STATE_PRESSED,      // ����״̬
    KEY_STATE_RELEASED,     // �ͷ�״̬
    KEY_STATE_LONG_PRESS    // ����״̬
} KeyStateTypeDef;

// �����¼�ö��
typedef enum {
    KEY_EVENT_NONE,         // ���¼�
    KEY_EVENT_CLICK,        // �����¼�
    KEY_EVENT_DOUBLE_CLICK, // ˫���¼�
    KEY_EVENT_LONG_PRESS    // �����¼�
} KeyEventTypeDef;

// �����ṹ�嶨��
typedef struct {
    GPIO_TypeDef* gpio_port;    // ����GPIO�˿�
    uint16_t gpio_pin;          // ����GPIO����
    KeyStateTypeDef state;      // ��ǰ״̬
    uint32_t press_time;        // ����ʱ���
    uint32_t release_time;      // �ͷ�ʱ���
    uint8_t click_count;        // �������
    uint8_t long_press_flag;    // ������־
    KeyEventTypeDef event;      // �����¼�
		uint32_t state_change_time;  // ״̬�仯��ʱ���������������
    uint8_t stable_pin_state;    // ������������ȶ�����״̬
} KeyTypeDef;


extern uint8_t task_running;    // �������б�־
extern uint8_t current_mode;    // ��ǰģʽ
extern uint16_t step_mode_1;
extern uint16_t step_mode_2;
extern uint16_t step_mode_3;
extern uint16_t step_mode_4;
extern uint8_t zero_init;
//extern uint8_t data_logging;

extern uint8_t circle_phase;  // 0:δ��ʼ 1:ֱ�ߵ�Բ����� 2:ִ��Բ���˶�

// ��������
void Key_Init(void);
void Key_Process(void);
void Task_Execute(void);
uint8_t Key_GetTaskState(void);
uint8_t Key_GetCurrentMode(void);
KeyTypeDef Key1_scope(void);
KeyTypeDef Key2_scope(void);
KeyTypeDef Key3_scope(void);
KeyTypeDef Key4_scope(void);


#endif /* __KEY_STATE_MACHINE_H */

