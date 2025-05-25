#ifndef __DCMOTOR_H
#define __DCMOTOR_H

#include "stm32f4xx.h"

/* ֹͣ���Ų����궨�� 
 * �����ſ���H���Ƿ���Ч�Դﵽ�����͹رյ����Ч��
 */
#define SHUTDOWN1_Pin                 GPIO_PIN_10
#define SHUTDOWN1_GPIO_Port           GPIOE

#define SHUTDOWN2_Pin                 GPIO_PIN_2
#define SHUTDOWN2_GPIO_Port           GPIOE
#define SHUTDOWN_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)   /* PF��ʱ��ʹ�� */

/* ���ֹͣ���Ŷ��� ����Ĭ���ǽӿ�1 */
#define ENABLE_MOTOR    HAL_GPIO_WritePin(SHUTDOWN1_GPIO_Port,SHUTDOWN1_Pin,GPIO_PIN_SET)
#define DISABLE_MOTOR   HAL_GPIO_WritePin(SHUTDOWN1_GPIO_Port,SHUTDOWN1_Pin,GPIO_PIN_RESET)

/******************************************************************************************/

void dcmotor_init(void);                /* ֱ����ˢ�����ʼ�� */
void dcmotor_start(void);               /* ������� */
void dcmotor_stop(void);                /* �رյ�� */  
void dcmotor_dir(uint8_t para);         /* ���õ������ */
void dcmotor_speed(uint16_t para);      /* ���õ���ٶ� */
void motor_pwm_set(float para);         /* ������� */
    
#endif


















