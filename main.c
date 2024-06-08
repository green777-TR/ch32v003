
//#include "MPU.h"
/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/08/08
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*
 *@Note
 *two-wire full duplex mode, master/slave mode, data transceiver:
 *Master:SPI1_SCK(PC5)銆丼PI1_MISO(PC7)銆丼PI1_MOSI(PC6).
 *Slave:SPI1_SCK(PC5)銆丼PI1_MISO(PC7)銆丼PI1_MOSI(PC6).
 *
 *This example demonstrates simultaneous full-duplex transmission and reception
 *between Master and Slave.
 *Note: The two boards download the Master and Slave programs respectively,
 *and power on at the same time.
 *Hardware connection:PC5 -- PC5
 *                      PC6 -- PC6
 *                      PC7 -- PC7
 *
 */


#include "debug.h"
#include "string.h"
#include "NRF24L01.h"


void SPISendReceiveBytes(uint8_t *sendData, uint8_t *getData, uint32_t length);
void SPISendBytes(uint8_t *sendData, uint32_t length);
void SPIReceiveBytes(uint8_t *getData, uint32_t length);
/* SPI Mode Definition */
#define HOST_MODE   0
#define SLAVE_MODE   1

/* SPI Communication Mode Selection */
#define SPI_MODE   HOST_MODE
//#define SPI_MODE   SLAVE_MODE

/* Global define */
#define Size 32

/* Global Variable */


/*********************************************************************
 * @fn      SPI_FullDuplex_Init
 *
 * @brief   Configuring the SPI for full-duplex communication.
 *
 * @return  none
 */
/*
#define SPI_CSN_PIN                  GPIO_Pin_3
#define SPI_CSN_PORT                 GPIOC
#define SPI_CE_PIN                  GPIO_Pin_2
#define SPI_CE_PORT                 GPIOD
*/
void SPI_FullDuplex_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    SPI_InitTypeDef SPI_InitStructure={0};


    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_Init( GPIOD, &GPIO_InitStructure );

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC | RCC_APB2Periph_SPI1, ENABLE );
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_Init( GPIOC, &GPIO_InitStructure );

#if (SPI_MODE == HOST_MODE)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

#elif (SPI_MODE == SLAVE_MODE)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

#endif

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;

#if (SPI_MODE == HOST_MODE)
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_SSOutputCmd(SPI1, ENABLE);


#elif (SPI_MODE == SLAVE_MODE)
    SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;

#endif

    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init( SPI1, &SPI_InitStructure );

    SPI_Cmd( SPI1, ENABLE );
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
#define SPI_DELAY                   100

void SPISendBytes(uint8_t *sendData, uint32_t length)
{
    uint32_t loop = 0;
    uint8_t tmp = 0;
    for(loop = 0; loop < length; loop++)
    {
        //Send SPI Byte
        while( SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_TXE ) == RESET ); // wait while flag is zero or TX buffer not empty
        SPI_I2S_SendData( SPI1, sendData[loop] );

        //Receive SPI Byte
        while(SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_RXNE ) == RESET ); // wait while flag is zero or RX buffer is empty
       tmp = SPI_I2S_ReceiveData( SPI1 );
    }

}

void SPIReceiveBytes(uint8_t *getData, uint32_t length)
{
    uint32_t loop = 0;
    for(loop = 0; loop < length; loop++)
    {
        //Send SPI Byte
        while( SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_TXE ) == RESET ); // wait while flag is zero or TX buffer not empty
        SPI_I2S_SendData( SPI1, 0x00 );

        //Receive SPI Byte
        while(SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_RXNE ) == RESET ); // wait while flag is zero or RX buffer is empty
        getData[loop] = SPI_I2S_ReceiveData( SPI1 );

    }

}

void SPISendReceiveBytes(uint8_t *sendData, uint8_t *getData, uint32_t length)
{
    uint32_t loop = 0;
    for(loop = 0; loop < length; loop++)
    {
        //Send SPI Byte
        while( SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_TXE ) == RESET ); // wait while flag is zero or TX buffer not empty
        SPI_I2S_SendData( SPI1, sendData[loop] );

        //Receive SPI Byte
        while(SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_RXNE ) == RESET ); // wait while flag is zero or RX buffer is empty
        getData[loop] = SPI_I2S_ReceiveData( SPI1 );

    }
}
/*
void SPIFlashReadID(uint8_t *spiFlashID)
{

    uint8_t txBuf[4] = {0};

    txBuf[0] = 0x9F;
    txBuf[1] = 0x00;
    txBuf[2] = 0x00;
    txBuf[3] = 0x00;

    GPIO_ResetBits(SPI_CSN_PORT, SPI_CSN_PIN); // CS LOW

    SPISendReceiveBytes(txBuf, spiFlashID, 4);

    GPIO_SetBits(SPI_CSN_PORT, SPI_CSN_PIN); // CS HIGH
}
*/




//uint8_t MyAddress[] = { 0x00,0xDD,0xCC,0xBB,0xAA }; // Địa chỉ của node phát
//uint8_t TxAddress[] = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 }; // Địa chỉ của node nhận
//uint8_t data[50];
u16 TxAddres[Size] = { 0x00,0xDD,0xCC,0xBB,0xAA };
u16 MyAddress[Size];
//uint8_t data_array[6];
int i = 5;
int main(void)
{
    SystemInit();
    Delay_Init();
    USART_Printf_Init(115200);
    SPI_FullDuplex_Init();
    printf("NRF24L01 Receiver Test\r\n");
    NRF24_Init();
    Delay_Ms(100);
    uint8_t address[5] = {0x01, 0x02, 0x03, 0x04, 0x05};

    NRF24_TxMode(address, 40);
    uint8_t data[32] = "Hello";

    //NRF24_RxMode(address, 40);
    //uint8_t data[32];
    uint8_t config_value = nrf24_ReadReg(CONFIG);
    printf("CONFIG register value: 0x%02X\r\n", config_value);
    while( 1 )
    {
        NRF24_Transmit(data);
           Delay_Ms(100);
       /* if (isDataAvailable(0)==1)
                    {
                        // Đọc dữ liệu
                        NRF24_Receive(data);
                        for (int i = 0; data[i] != '\0'; i++)
                                                  {
                                                          printf("%c", data[i]);
                                                          Delay_Ms(1000);

                                                  }

                        // Xử lý dữ liệu tại đây...
                    }
*/

           if (NRF24_Transmit(data)==1) {
               // Dữ liệu đã được gửi thành công
               printf("Data sent successfully!\r\n");
              // nrf24_WriteReg(STATUS, (1 << TX_DS)); // Xóa bit TX_DS
           } else if (NRF24_Transmit(data)==0) {
               // Gửi dữ liệu thất bại
               printf("Data transmission failed!\r\n");
              // nrf24_WriteReg(STATUS, (1 << MAX_RT)); // Xóa bit MAX_RT
           }Delay_Ms(100);
    }
}
/**********************************************************************
**********************************************************************/
/*
int main(void)
{

    Delay_Init();
    USART_Printf_Init(115200);

    printf("\r\n\r\n: MPU6050 003 :\r\n");
     printf("- SystemClk:%d\r\n",SystemCoreClock);

    MPU6050_Init();

     Delay_Ms(500);

    MPU6050_Write();

     Delay_Ms(500);
     short aacx,aacy,aacz;

    while( 1 )
    {

        //MPU6050_TEST();

        MPU_Get_Accelerometer(&aacx,&aacy,&aacz);
        printf("ACC  : X=%d , Y=%d , Z=%d  \r\n", aacx , aacy , aacz );

         Delay_Ms(1000);

    }

}
*/
