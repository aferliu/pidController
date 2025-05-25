/**
 ****************************************************************************************************
 * @file        dcmotor_time.c
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

#include "./BSP/TIMER/dcmotor_tim.h"
#include "./BSP/DC_MOTOR/dc_motor.h"


/******************************* ��һ����  ����������� ����������������� **************************************/


TIM_HandleTypeDef g_atimx_cplm_pwm_handle;                              /* ��ʱ��x��� */

/**
 * @brief       �߼���ʱ��TIMX ������� ��ʼ��������ʹ��PWMģʽ1��
 * @note
 *              ���ø߼���ʱ��TIMX �������, һ·OCy һ·OCyN, ���ҿ�����������ʱ��
 *
 *              �߼���ʱ����ʱ������APB2, ��PCLK2 = 168Mhz, ��������PPRE2����Ƶ, ���
 *              �߼���ʱ��ʱ�� = 168Mhz
 *              ��ʱ�����ʱ����㷽��: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=��ʱ������Ƶ��, ��λ : Mhz
 *
 * @param       arr: �Զ���װֵ��
 * @param       psc: ʱ��Ԥ��Ƶ��
 * @retval      ��
 */

void atim_timx_cplm_pwm_init(uint16_t arr, uint16_t psc)
{
    TIM_OC_InitTypeDef sConfigOC ;
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

    g_atimx_cplm_pwm_handle.Instance = ATIM_TIMX_CPLM;                      /* ��ʱ��x */
    g_atimx_cplm_pwm_handle.Init.Prescaler = psc;                           /* ��ʱ��Ԥ��Ƶϵ�� */
    g_atimx_cplm_pwm_handle.Init.CounterMode = TIM_COUNTERMODE_UP;          /* ���ϼ���ģʽ */
    g_atimx_cplm_pwm_handle.Init.Period = arr;                              /* �Զ���װ��ֵ */
    g_atimx_cplm_pwm_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;    /* ʱ�ӷ�Ƶ���� */
    g_atimx_cplm_pwm_handle.Init.RepetitionCounter = 0;                     /* �ظ��������Ĵ���Ϊ0 */
    g_atimx_cplm_pwm_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;        /* ʹ��Ӱ�ӼĴ���TIMx_ARR */
    HAL_TIM_PWM_Init(&g_atimx_cplm_pwm_handle) ;

    /* ����PWM��� */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;                                     /* PWMģʽ1 */
    sConfigOC.Pulse = 0;                                                    /* �Ƚ�ֵΪ0 */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;                              /* OCy �͵�ƽ��Ч */
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;                            /* OCyN �͵�ƽ��Ч */
    sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;                               /* ��ʹ�ÿ���ģʽ */
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;                          /* ��ͨ���Ŀ���״̬ */
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;                        /* ����ͨ���Ŀ���״̬ */
    HAL_TIM_PWM_ConfigChannel(&g_atimx_cplm_pwm_handle, &sConfigOC, ATIM_TIMX_CPLM_CHY);    /* ���ú�Ĭ����CCER�Ļ������λ */   
    
    /* �����������������������ж� */
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;                 /* OSSR����Ϊ1 */
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;               /* OSSI����Ϊ0 */
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;                     /* �ϵ�ֻ��дһ�Σ���Ҫ��������ʱ��ʱֻ���ô�ֵ */
    sBreakDeadTimeConfig.DeadTime = 0X0F;                                   /* ����ʱ�� */
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;                    /* BKE = 0, �ر�BKIN��� */
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;             /* BKP = 1, BKIN�͵�ƽ��Ч */
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;     /* ʹ��AOEλ������ɲ�����Զ��ָ���� */
    HAL_TIMEx_ConfigBreakDeadTime(&g_atimx_cplm_pwm_handle, &sBreakDeadTimeConfig);         /* ����BDTR�Ĵ��� */

}

/**
 * @brief       ��ʱ���ײ�������ʱ��ʹ�ܣ���������
                �˺����ᱻHAL_TIM_PWM_Init()����
 * @param       htim:��ʱ�����
 * @retval      ��
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == ATIM_TIMX_CPLM)
    {
        GPIO_InitTypeDef gpio_init_struct;
        
        ATIM_TIMX_CPLM_CHY_GPIO_CLK_ENABLE();   /* ͨ��X��ӦIO��ʱ��ʹ�� */
        ATIM_TIMX_CPLM_CHYN_GPIO_CLK_ENABLE();  /* ͨ��X����ͨ����ӦIO��ʱ��ʹ�� */
        ATIM_TIMX_CPLM_CLK_ENABLE();            /* ��ʱ��xʱ��ʹ�� */

        /* ����PWM��ͨ������ */
        gpio_init_struct.Pin = ATIM_TIMX_CPLM_CHY_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_NOPULL;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH ;
        gpio_init_struct.Alternate = ATIM_TIMX_CPLM_CHY_GPIO_AF;                /* �˿ڸ��� */   
        HAL_GPIO_Init(ATIM_TIMX_CPLM_CHY_GPIO_PORT, &gpio_init_struct);

        /* ����PWM����ͨ������ */
        gpio_init_struct.Pin = ATIM_TIMX_CPLM_CHYN_GPIO_PIN;
        HAL_GPIO_Init(ATIM_TIMX_CPLM_CHYN_GPIO_PORT, &gpio_init_struct);
    }
}


/******************************* �ڶ�����  ������������� ****************************************************/

/********************************* 1 ͨ�ö�ʱ�� ���������� *************************************/

TIM_HandleTypeDef g_timx_encode_chy_handle;         /* ��ʱ��x��� */
TIM_Encoder_InitTypeDef g_timx_encoder_chy_handle;  /* ��ʱ����������� */

/**
 * @brief       ͨ�ö�ʱ��TIMX ͨ��Y �������ӿ�ģʽ ��ʼ������
 * @note
 *              ͨ�ö�ʱ����ʱ������APB1,��PPRE1 �� 2��Ƶ��ʱ��
 *              ͨ�ö�ʱ����ʱ��ΪAPB1ʱ�ӵ�2��, ��APB1Ϊ42M, ���Զ�ʱ��ʱ�� = 84Mhz
 *              ��ʱ�����ʱ����㷽��: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=��ʱ������Ƶ��,��λ:Mhz
 *
 * @param       arr: �Զ���װֵ��
 * @param       psc: ʱ��Ԥ��Ƶ��
 * @retval      ��
 */
void gtim_timx_encoder_chy_init(uint16_t arr, uint16_t psc)
{
    /* ��ʱ��x���� */
    g_timx_encode_chy_handle.Instance = GTIM_TIMX_ENCODER;                      /* ��ʱ��x */
    g_timx_encode_chy_handle.Init.Prescaler = psc;                              /* ��ʱ����Ƶ */
    g_timx_encode_chy_handle.Init.Period = arr;                                 /* �Զ���װ��ֵ */
    g_timx_encode_chy_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;       /* ʱ�ӷ�Ƶ���� */
    
    /* ��ʱ��x���������� */
    g_timx_encoder_chy_handle.EncoderMode = TIM_ENCODERMODE_TI12;               /* TI1��TI2����⣬4��Ƶ */
    g_timx_encoder_chy_handle.IC1Polarity = TIM_ICPOLARITY_RISING;              /* ���뼫�ԣ��Ƿ��� */
    g_timx_encoder_chy_handle.IC1Selection = TIM_ICSELECTION_DIRECTTI;          /* ����ͨ��ѡ�� */
    g_timx_encoder_chy_handle.IC1Prescaler = TIM_ICPSC_DIV1;                    /* ����Ƶ */
    g_timx_encoder_chy_handle.IC1Filter = 10;                                   /* �˲������� */
    g_timx_encoder_chy_handle.IC2Polarity = TIM_ICPOLARITY_RISING;              /* ���뼫�ԣ��Ƿ��� */
    g_timx_encoder_chy_handle.IC2Selection = TIM_ICSELECTION_DIRECTTI;          /* ����ͨ��ѡ�� */
    g_timx_encoder_chy_handle.IC2Prescaler = TIM_ICPSC_DIV1;                    /* ����Ƶ */
    g_timx_encoder_chy_handle.IC2Filter = 10;                                   /* �˲������� */
    HAL_TIM_Encoder_Init(&g_timx_encode_chy_handle, &g_timx_encoder_chy_handle);/* ��ʼ����ʱ��x������ */
     
    HAL_TIM_Encoder_Start(&g_timx_encode_chy_handle,GTIM_TIMX_ENCODER_CH1);     /* ʹ�ܱ�����ͨ��1 */
    HAL_TIM_Encoder_Start(&g_timx_encode_chy_handle,GTIM_TIMX_ENCODER_CH2);     /* ʹ�ܱ�����ͨ��2 */
    __HAL_TIM_ENABLE_IT(&g_timx_encode_chy_handle,TIM_IT_UPDATE);               /* ʹ�ܸ����ж� */
    __HAL_TIM_CLEAR_FLAG(&g_timx_encode_chy_handle,TIM_IT_UPDATE);              /* ��������жϱ�־λ */
    
}

/**
 * @brief       ��ʱ���ײ�������ʱ��ʹ�ܣ���������
                �˺����ᱻHAL_TIM_Encoder_Init()����
 * @param       htim:��ʱ�����
 * @retval      ��
 */
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == GTIM_TIMX_ENCODER)
    {
        GPIO_InitTypeDef gpio_init_struct;
        GTIM_TIMX_ENCODER_CH1_GPIO_CLK_ENABLE();                                 /* ����ͨ��y��GPIOʱ�� */
        GTIM_TIMX_ENCODER_CH2_GPIO_CLK_ENABLE();
        GTIM_TIMX_ENCODER_CH1_CLK_ENABLE();                                      /* ������ʱ��ʱ�� */
        GTIM_TIMX_ENCODER_CH2_CLK_ENABLE();

        gpio_init_struct.Pin = GTIM_TIMX_ENCODER_CH1_GPIO_PIN;                   /* ͨ��y��GPIO�� */
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                                 /* ����������� */
        gpio_init_struct.Pull = GPIO_NOPULL;                                     /* ���� */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;                           /* ���� */
        gpio_init_struct.Alternate = GTIM_TIMX_ENCODERCH1_GPIO_AF;               /* �˿ڸ��� */
        HAL_GPIO_Init(GTIM_TIMX_ENCODER_CH1_GPIO_PORT, &gpio_init_struct);  
        
        gpio_init_struct.Pin = GTIM_TIMX_ENCODER_CH2_GPIO_PIN;                   /* ͨ��y��GPIO�� */
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                                 /* ����������� */
        gpio_init_struct.Pull = GPIO_NOPULL;                                     /* ���� */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;                           /* ���� */
        gpio_init_struct.Alternate = GTIM_TIMX_ENCODERCH2_GPIO_AF;               /* �˿ڸ��� */
        HAL_GPIO_Init(GTIM_TIMX_ENCODER_CH2_GPIO_PORT, &gpio_init_struct);         
       
        HAL_NVIC_SetPriority(GTIM_TIMX_ENCODER_INT_IRQn, 2, 0);                  /* �ж����ȼ����� */
        HAL_NVIC_EnableIRQ(GTIM_TIMX_ENCODER_INT_IRQn);                          /* �����ж� */
    }
}

/**
 * @brief       ��ʱ���жϷ�����
 * @param       ��
 * @retval      ��
 */
void GTIM_TIMX_ENCODER_INT_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_timx_encode_chy_handle);
}


/******************************** 2 ������ʱ�� ���������� ************************************/

TIM_HandleTypeDef timx_handler;         /* ��ʱ��������� */

/**
 * @brief       ������ʱ��TIMX��ʱ�жϳ�ʼ������
 * @note
 *              ������ʱ����ʱ������APB1,��PPRE1 �� 2��Ƶ��ʱ��
 *              ������ʱ����ʱ��ΪAPB1ʱ�ӵ�2��, ��APB1Ϊ42M, ���Զ�ʱ��ʱ�� = 84Mhz
 *              ��ʱ�����ʱ����㷽��: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=��ʱ������Ƶ��,��λ:Mhz
 *
 * @param       arr: �Զ���װֵ��
 * @param       psc: ʱ��Ԥ��Ƶ��
 * @retval      ��
 */
void btim_timx_int_init(uint16_t arr, uint16_t psc)
{
    timx_handler.Instance = BTIM_TIMX_INT;                              /* ������ʱ��X */
    timx_handler.Init.Prescaler = psc;                                  /* ����Ԥ��Ƶ��  */
    timx_handler.Init.CounterMode = TIM_COUNTERMODE_UP;                 /* ���ϼ����� */
    timx_handler.Init.Period = arr;                                     /* �Զ�װ��ֵ */
    timx_handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;           /* ʱ�ӷ�Ƶ���� */
    HAL_TIM_Base_Init(&timx_handler);
    
    HAL_TIM_Base_Start_IT(&timx_handler);                               /* ʹ�ܻ�����ʱ��x�ͼ�������жϣ�TIM_IT_UPDATE */
    __HAL_TIM_CLEAR_IT(&timx_handler,TIM_IT_UPDATE);                    /* ��������жϱ�־λ */
}

/**
 * @brief       ��ʱ���ײ�����������ʱ�ӣ������ж����ȼ�
                �˺����ᱻHAL_TIM_Base_Init()��������
 * @param       ��
 * @retval      ��
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == BTIM_TIMX_INT)
    {
        BTIM_TIMX_INT_CLK_ENABLE();                                     /*ʹ��TIMʱ��*/
        HAL_NVIC_SetPriority(BTIM_TIMX_INT_IRQn, 1, 3);                 /* ��ռ1�������ȼ�3����2 */
        HAL_NVIC_EnableIRQ(BTIM_TIMX_INT_IRQn);                         /*����ITM3�ж�*/
    }
}

/**
 * @brief       ������ʱ��TIMX�жϷ�����
 * @param       ��
 * @retval      ��
 */
void BTIM_TIMX_INT_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&timx_handler);                                  /*��ʱ���ص�����*/
}


/******************************** 3 ���ò��� ���������� ************************************/

volatile int g_timx_encode_count = 0;                                   /* ������� */

/**
 * @brief       ��ʱ�������жϻص�����
 * @param        htim:��ʱ�����ָ��
 * @note        �˺����ᱻ��ʱ���жϺ�����ͬ���õ�
 * @retval      ��
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
        if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&g_timx_encode_chy_handle))   /* �ж�CR1��DIRλ */
        {
            g_timx_encode_count--;                                      /* DIRλΪ1��Ҳ���ǵݼ����� */
        }
        else
        {
            g_timx_encode_count++;                                      /* DIRλΪ0��Ҳ���ǵ������� */
        }
    }
    else if (htim->Instance == TIM6)
    {
        int Encode_now = gtim_get_encode();                             /* ��ȡ������ֵ�����ڼ����ٶ� */

        // speed_computer(Encode_now, 50);                                 /* ��λƽ��ֵ�˳��������������ݣ�50ms����һ���ٶ�*/
    }
}

/**
 * @brief       ��ȡ��������ֵ
 * @param       ��
 * @retval      ������ֵ
 */
int gtim_get_encode(void)
{
    return ( int32_t )__HAL_TIM_GET_COUNTER(&g_timx_encode_chy_handle) + g_timx_encode_count * 65536;       /* ��ǰ����ֵ+֮ǰ�ۼƱ�������ֵ=�ܵı�����ֵ */
}


