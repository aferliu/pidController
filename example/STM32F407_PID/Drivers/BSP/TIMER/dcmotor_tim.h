/**
 ****************************************************************************************************
 * @file        dcmotor_tim.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-19
 * @brief       ��ʱ�� ��������
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� F407���������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com/forum.php
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 * �޸�˵��
 * V1.0 20211019
 * ��һ�η���
 *
 ****************************************************************************************************
 */

#ifndef __DCMOTOR_TIM_H
#define __DCMOTOR_TIM_H

#include "./SYSTEM/sys/sys.h"

/******************************* ��һ����  ����������� ����������������� **************************************/

/* TIMX �������ģʽ���� */

/* ���ͨ������ */
#define ATIM_TIMX_CPLM_CHY_GPIO_PORT            GPIOA
#define ATIM_TIMX_CPLM_CHY_GPIO_PIN             GPIO_PIN_8
#define ATIM_TIMX_CPLM_CHY_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PA��ʱ��ʹ�� */

/* �������ͨ������ */
#define ATIM_TIMX_CPLM_CHYN_GPIO_PORT           GPIOB
#define ATIM_TIMX_CPLM_CHYN_GPIO_PIN            GPIO_PIN_13
#define ATIM_TIMX_CPLM_CHYN_GPIO_CLK_ENABLE()   do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* PB��ʱ��ʹ�� */

/* TIMX ���Ÿ�������
 * ��ΪPA8/PB13, Ĭ�ϲ�����TIM1�Ĺ��ܽ�, ���뿪������, �ſ���: TIM1_CH1->PA8; TIM1_CH1N->PB13;
 * ����,PA8/PB13��������TIM1��CH1/CH1N����.
 */
#define ATIM_TIMX_CPLM_CHY_GPIO_AF              GPIO_AF1_TIM1

/* �������ʹ�õĶ�ʱ�� */
#define ATIM_TIMX_CPLM                          TIM1
#define ATIM_TIMX_CPLM_CHY                      TIM_CHANNEL_1
#define ATIM_TIMX_CPLM_CHY_CCRY                 ATIM_TIMX_CPLM->CCR1
#define ATIM_TIMX_CPLM_CLK_ENABLE()             do{ __HAL_RCC_TIM1_CLK_ENABLE(); }while(0)    /* TIM1 ʱ��ʹ�� */


/******************************* �ڶ�����  ������������� **************************************/

/* ͨ�ö�ʱ�� ����  */
#define GTIM_TIMX_ENCODER_CH1_GPIO_PORT         GPIOB
#define GTIM_TIMX_ENCODER_CH1_GPIO_PIN          GPIO_PIN_6
#define GTIM_TIMX_ENCODER_CH1_GPIO_CLK_ENABLE() do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)  /* PC��ʱ��ʹ�� */

#define GTIM_TIMX_ENCODER_CH2_GPIO_PORT         GPIOB
#define GTIM_TIMX_ENCODER_CH2_GPIO_PIN          GPIO_PIN_7
#define GTIM_TIMX_ENCODER_CH2_GPIO_CLK_ENABLE() do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)  /* PC��ʱ��ʹ�� */

/* TIMX ���Ÿ�������
 * ��ΪPC6/PC7, Ĭ�ϲ�����TIM3�Ĺ��ܽ�, ���뿪������, �ſ�������TIM3��CH1/CH2����
 */
#define GTIM_TIMX_ENCODERCH1_GPIO_AF            GPIO_AF2_TIM4                                /* �˿ڸ��õ�TIM3 */
#define GTIM_TIMX_ENCODERCH2_GPIO_AF            GPIO_AF2_TIM4                                /* �˿ڸ��õ�TIM3 */

#define GTIM_TIMX_ENCODER                       TIM4                                         /* TIM3 */
#define GTIM_TIMX_ENCODER_INT_IRQn              TIM4_IRQn
#define GTIM_TIMX_ENCODER_INT_IRQHandler        TIM4_IRQHandler

#define GTIM_TIMX_ENCODER_CH1                   TIM_CHANNEL_1                                /* ͨ��1 */
#define GTIM_TIMX_ENCODER_CH1_CLK_ENABLE()      do{ __HAL_RCC_TIM4_CLK_ENABLE(); }while(0)   /* TIM3 ʱ��ʹ�� */

#define GTIM_TIMX_ENCODER_CH2                   TIM_CHANNEL_2                                /* ͨ��2 */
#define GTIM_TIMX_ENCODER_CH2_CLK_ENABLE()      do{ __HAL_RCC_TIM4_CLK_ENABLE(); }while(0)   /* TIM3 ʱ��ʹ�� */


/* ������ʱ�� ���� 
 * ���������ֵ�����ڼ����ٶ�
 */
#define BTIM_TIMX_INT                           TIM6
#define BTIM_TIMX_INT_IRQn                      TIM6_DAC_IRQn
#define BTIM_TIMX_INT_IRQHandler                TIM6_DAC_IRQHandler
#define BTIM_TIMX_INT_CLK_ENABLE()              do{ __HAL_RCC_TIM6_CLK_ENABLE(); }while(0)    /* TIM6 ʱ��ʹ�� */

/* �����������ṹ�� */
typedef struct 
{
  int encode_old;                  /* ��һ�μ���ֵ */
  int encode_now;                  /* ��ǰ����ֵ */
  float speed;                     /* �������ٶ� */
} ENCODE_TypeDef;

extern ENCODE_TypeDef g_encode;    /* �������������� */

/******************************************************************************************/

void atim_timx_cplm_pwm_init(uint16_t arr, uint16_t psc);           /* �߼���ʱ�� ���������ʼ�� */
void gtim_timx_encoder_chy_init(uint16_t arr, uint16_t psc);        /* ͨ�ö�ʱ�� ��ʱ�жϳ�ʼ�� */
void btim_timx_int_init(uint16_t arr, uint16_t psc);                /* ������ʱ�� ��ʱ�жϳ�ʼ�� */
int gtim_get_encode(void);                                          /* ��ȡ�������ܼ���ֵ */

#endif




