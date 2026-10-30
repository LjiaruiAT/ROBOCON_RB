/**
 * @file    imu_probe.c
 * @brief   四串口同时监听，用于定位 IMU 实际接在哪个 UART
 */
#include "imu_probe.h"
#include "JY61.h"

uint8_t g_probe_buf4[64];
uint8_t g_probe_buf6[64];
uint8_t g_probe_buf2[64];
uint8_t g_probe_buf3[64];

volatile uint32_t g_probe_cb4 = 0u, g_probe_cb6 = 0u, g_probe_cb2 = 0u, g_probe_cb3 = 0u;
volatile uint32_t g_probe_n4  = 0u, g_probe_n6  = 0u, g_probe_n2  = 0u, g_probe_n3  = 0u;
volatile uint32_t g_probe_e4  = 0u, g_probe_e6  = 0u, g_probe_e2  = 0u, g_probe_e3  = 0u;
volatile uint32_t g_probe_total = 0u;

/* USART3 在本工程里没有被 CubeMX 配置，这里手工建句柄和 DMA */
UART_HandleTypeDef huart3;
static DMA_HandleTypeDef hdma_usart3_rx;

static void probe_start(UART_HandleTypeDef *h, uint8_t *buf)
{
    if (h->hdmarx == NULL)
    {
        return;
    }
    __HAL_UART_ENABLE_IT(h, UART_IT_IDLE);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(h, buf, 64u);
    __HAL_DMA_DISABLE_IT(h->hdmarx, DMA_IT_HT);
}

/* PD8/PD9 = USART3，参考工程里有、本工程原本没配 */
static void usart3_init_manual(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART3;

    gpio.Pin = GPIO_PIN_9;              /* RX */
    HAL_GPIO_Init(GPIOD, &gpio);
    gpio.Pin = GPIO_PIN_8;              /* TX */
    HAL_GPIO_Init(GPIOD, &gpio);

    huart3.Instance          = USART3;
    huart3.Init.BaudRate     = 115200;
    huart3.Init.WordLength   = UART_WORDLENGTH_8B;
    huart3.Init.StopBits     = UART_STOPBITS_1;
    huart3.Init.Parity       = UART_PARITY_NONE;
    huart3.Init.Mode         = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        return;
    }

    hdma_usart3_rx.Instance                 = DMA1_Stream1;
    hdma_usart3_rx.Init.Channel             = DMA_CHANNEL_4;
    hdma_usart3_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode                = DMA_NORMAL;
    hdma_usart3_rx.Init.Priority            = DMA_PRIORITY_LOW;
    hdma_usart3_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
    {
        return;
    }
    __HAL_LINKDMA(&huart3, hdmarx, hdma_usart3_rx);

    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
}

void ImuProbe_Init(void)
{
    usart3_init_manual();

    probe_start(&huart4, g_probe_buf4);
    probe_start(&huart6, g_probe_buf6);
    probe_start(&huart2, g_probe_buf2);
    probe_start(&huart3, g_probe_buf3);
}

/* 找到端口对应的缓冲；不是候选口就返回 NULL */
static uint8_t *probe_buf_of(UART_HandleTypeDef *h)
{
    if (h->Instance == UART4)  { return g_probe_buf4; }
    if (h->Instance == USART6) { return g_probe_buf6; }
    if (h->Instance == USART2) { return g_probe_buf2; }
    if (h->Instance == USART3) { return g_probe_buf3; }
    return NULL;
}

uint8_t ImuProbe_OnRx(UART_HandleTypeDef *huart, uint16_t size)
{
    uint8_t *buf = probe_buf_of(huart);

    if (buf == NULL)
    {
        return 0u;
    }

    if (huart->Instance == UART4)       { g_probe_cb4++; g_probe_n4 += size; }
    else if (huart->Instance == USART6) { g_probe_cb6++; g_probe_n6 += size; }
    else if (huart->Instance == USART2) { g_probe_cb2++; g_probe_n2 += size; }
    else                                { g_probe_cb3++; g_probe_n3 += size; }
    g_probe_total++;

    /* 只有选中的那个口才喂给 JY61 解析器 */
    if (huart->Instance == IMU_HUART.Instance)
    {
        JY61_Receive(&JY61, buf, (uint8_t)size);
    }

    HAL_UART_DMAStop(huart);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(huart, buf, 64u);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    return 1u;
}

uint8_t ImuProbe_OnErr(UART_HandleTypeDef *huart)
{
    uint8_t *buf = probe_buf_of(huart);
    uint32_t isrflags;

    if (buf == NULL)
    {
        return 0u;
    }

    if (huart->Instance == UART4)       { g_probe_e4++; }
    else if (huart->Instance == USART6) { g_probe_e6++; }
    else if (huart->Instance == USART2) { g_probe_e2++; }
    else                                { g_probe_e3++; }

    HAL_UART_DMAStop(huart);

    /* 先读 SR 再读 DR 才能清 ORE/NE/FE */
    isrflags = READ_REG(huart->Instance->SR);
    if (isrflags & (USART_SR_ORE | USART_SR_NE | USART_SR_FE))
    {
        volatile uint32_t tmp_sr = READ_REG(huart->Instance->SR);
        volatile uint32_t tmp_dr = READ_REG(huart->Instance->DR);
        (void)tmp_sr;
        (void)tmp_dr;
    }

    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->RxState   = HAL_UART_STATE_READY;
    huart->gState    = HAL_UART_STATE_READY;

    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(huart, buf, 64u);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    return 1u;
}

/* USART3 的中断入口：本工程原本没有，放在这里避免动 CubeMX 生成的文件 */
void USART3_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart3);
}

void DMA1_Stream1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart3_rx);
}
