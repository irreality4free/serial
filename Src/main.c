/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/

//#pragma once 
 
#include <stdint.h>
#include <stddef.h> 
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

//int packet_wd_timeout;
//
//
//int uptime;
//int state;
//int PACKET_WATCHDOG_TIMEOUT;
//void send_state(void);








/*




*/ 

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define BUFFER_LEN 32
char str_rx[21];
uint32_t uart_buffer_cur_len =0;
#define  UART_BUFFER_LEN  32
uint32_t uart_buffer_write = 0;
uint8_t uart_buffer[UART_BUFFER_LEN];
uint32_t uart_buffer_read = 0;
char buffer[BUFFER_LEN];
uint8_t control_cmd;







#define CRC_SIZE 2
#define HEADER_SIZE 1
#define UAV_ADDRESS 1
#define BIM_ADDRESS 1
#define BIM_PACK_HEADER_SIZE 1
#define BIM_CONTROL_PACK_LEN 1

#define PROTOCOL_CRC_INIT 0xFFFF
#define PROTOCOL_CRC_VALIDATE 0xF0B8
#define PERIOD 50
#define MIN_PACK_NENGTH 28
#define ENGINE_INDEX 8
#define LEFT_OFS_INDEX 8
#define RIGHT_OFS_INDEX 9
#define ID_INDEX 6
#define SET_POSITION 1
#define BIM_ENABLE 3
#define MARKER 0xFEAA
#define ADDRESS 0
#define VERSION 1
#define BIM_STATUS_ID 2
#define NPOWER 1
#define BATTERY_LOW 1674.

#define BATTERY_RANGE 2.49
#define CONNECTION_LOST 5000
#define BREAK_TIMEOUT 2000

#define ADC_TO_VOLTAGE 37.70974124



#define DEADZONE 500
enum {OFF, ON, SAFEOFF};
enum {LEFT, RIGHT};
enum {DOWN, UP};

uint32_t runtime = 0;
uint32_t pot[2] = {0, 0};
uint8_t percPot1, percPot2, battery, temperature;
uint8_t left_ofs, right_ofs;
uint8_t message[16];
uint8_t engine = 0;
uint32_t badCRC = 0;
uint32_t coord[2] = {0, 0};
uint32_t targetCoord[2] = {0, 0};
uint32_t up[2] = {0, 0};
uint32_t lastConnect = 0;
uint8_t turnOff = 0;
uint32_t breakTimeout[2] = {0, 0};
uint32_t breakEnable[2] = {0, 0};

uint8_t pos_cmd[2] = {0,0};
uint8_t packet_wd_timeout = 50;
uint32_t uptime = 0;



/*
extern uint8_t buffer[256];
extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;
*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/*
enum { 

    PROTOCOL_CRC_INIT = 0xFFFF,     
		PROTOCOL_CRC_VALIDATE = 0xF0B8 
}; */
             

struct uav_bim_state_t {  
	uint8_t left_ofs;  
	uint8_t right_ofs; 
};

struct protocol_header_t
{
 uint16_t marker;
 uint16_t sz;
 uint8_t src;
 uint8_t dst;
 uint8_t id;
 uint8_t version;
	
};

struct uav_bim_status_t
{
 uint32_t uptime;
 uint32_t badcrc;
 uint8_t left_ofs;
 uint8_t right_ofs;
 uint8_t enabled;
 uint8_t npower;
 uint8_t power[];
};
struct uav_bim_control_t
{
 uint8_t engine;
};
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void safe_stop(){}
/*	
void crcGood(){
	
	#pragma pack(push,1)
	struct uav_bim_status_t {
		uint16_t marker;
		uint16_t length;
		uint8_t senderAddress;
		uint8_t targetAddress;
		uint8_t id;
		uint8_t version;
		
		uint32_t uptime;
		uint32_t badcrc;
		uint8_t left_ofs;
		uint8_t right_ofs;
		uint8_t enabled;
		uint8_t npower;
		uint8_t power[1];
		uint16_t crc;
	} answer;
	#pragma pack(pop)
	
	answer.marker=MARKER;
	answer.length = sizeof(answer);
	answer.senderAddress = ADDRESS;
	answer.targetAddress = ADDRESS;
	answer.id = BIM_STATUS_ID;
	answer.version = VERSION;
	
	answer.uptime = runtime;
	answer.badcrc = badCRC;
	answer.left_ofs = percPot1;
	answer.right_ofs = percPot2;
	answer.enabled = engine;
	answer.npower = NPOWER;
	answer.power[0] = battery;
	
	uint8_t * arr = (uint8_t *) &answer;
	answer.crc = protocol_crc_calc(arr, answer.length-2);
	HAL_UART_Transmit(&huart2, arr, answer.length, 100);
	
}

int send_pack(){
	#pragma pack(push,1)
	struct uav_bim_status_t {
		uint16_t marker;
		uint16_t length;
		uint8_t senderAddress;
		uint8_t targetAddress;
		uint8_t id;
		uint8_t version;
		
		uint32_t uptime;
		uint32_t badcrc;
		uint8_t left_ofs;
		uint8_t right_ofs;
		uint8_t enabled;
		uint8_t npower;
		uint8_t power[1];
		uint16_t crc;
	} pack;
	#pragma pack(pop)
	
	pack.marker = 			MARKER;
	pack.length = 			sizeof(pack);
	pack.senderAddress =	BIM_ADDRESS;
	pack.targetAddress = 	UAV_ADDRESS;
	pack.id = 				BIM_STATUS_PACK_ID;
	pack.version = 			BIM_STATUS_PACK_VERSION;
	
	pack.uptime = 			runtime;
	pack.badcrc = 			badCRC;
	pack.left_ofs = 		pos_fb[0];
	pack.right_ofs = 		pos_fb[1];
	pack.enabled = 			state;
	pack.npower = 			BIM_BATTERY_COUNT;
	for (uint8_t i=0; i<BIM_BATTERY_COUNT; i++)
		pack.power[i] = 		battery[i];
	
	uint8_t * arr = (uint8_t *) &pack;
	pack.crc = protocol_crc_calc(arr, pack.length-2);
	HAL_UART_Transmit(&huart2, arr, pack.length, 100);
}


	*/
/*	
void watchdog(){
	if (packet_wd_timeout > uptime && state==ENABLE){ safe_stop(); }
}
*/
int check_header(uint8_t n, uint8_t id, uint8_t version){
	struct protocol_header_t* h = (struct protocol_header_t*)(buffer + BUFFER_LEN - n - CRC_SIZE - BIM_PACK_HEADER_SIZE);
	char ans[] = "chekcheckcheck";
  CDC_Transmit_FS((unsigned char*) ans ,strlen(ans));	
	HAL_Delay(100);
	CDC_Transmit_FS("\n" , 1);
	HAL_Delay(100);
	
	CDC_Transmit_FS((uint8_t*)buffer ,BUFFER_LEN);
	HAL_Delay(100);
	CDC_Transmit_FS("\n" , 1);
	HAL_Delay(100);
	
	uint8_t mark_= h->marker;
	CDC_Transmit_FS(&mark_ ,2);
	HAL_Delay(100);
	CDC_Transmit_FS("\n" , 1);
	HAL_Delay(100);
	
	uint8_t sz_= h->sz;
	CDC_Transmit_FS(&sz_ ,2);
	HAL_Delay(100);
	CDC_Transmit_FS("\n" , 1);
	HAL_Delay(100);
	
	uint8_t src_= h->src;
	CDC_Transmit_FS(&src_ ,1);
	HAL_Delay(100);
	CDC_Transmit_FS("\n" , 1);
	HAL_Delay(100);
	
	uint8_t dst_= h->dst;
	CDC_Transmit_FS(&dst_ ,1);
	HAL_Delay(100);
	CDC_Transmit_FS("\n" , 1);
	HAL_Delay(100);
	
	uint8_t id_= h->id;
	CDC_Transmit_FS(&id_ ,1);
	HAL_Delay(100);
	CDC_Transmit_FS("\n" , 1);
	HAL_Delay(100);
	
	uint8_t version_= h->version;
	CDC_Transmit_FS(&version_ ,1);
	HAL_Delay(100);
	
	CDC_Transmit_FS("\n" , 1);
	HAL_Delay(100);
	
	
	
	
	
	if (h->marker != 0xFFAE) return 0;
	if (h->sz != n + CRC_SIZE + HEADER_SIZE) return 0;
	if (h->src != UAV_ADDRESS) return 0;
	if (h->dst != BIM_ADDRESS) return 0;
	if (h->id != id) return 0;
	if (h->version != version) return 0;
	char ans1[] = "found header";
  CDC_Transmit_FS((unsigned char*) ans1 ,strlen(ans));	
	return 0;
}

static inline void protocol_crc_acc(uint8_t v, uint16_t *pcrc)
{
 /* Accumulate one byte of data into the CRC */
 uint8_t tmp = v ^ (uint8_t)(*pcrc & 0xff);
 tmp ^= (tmp << 4);
 *pcrc = (*pcrc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

static inline void protocol_crc_acc_buf(
 uint16_t* pcrc, const void* buf, size_t sz)
{
 const uint8_t* p = (const uint8_t*)buf;
 while(sz--)
 protocol_crc_acc(*p++, pcrc);
}

static inline void protocol_crc_init(uint16_t* pcrc)
{
 *pcrc = PROTOCOL_CRC_INIT;
}

static inline uint16_t protocol_crc_calc(const void* buf, size_t sz)
{
 uint16_t crc;
 protocol_crc_init(&crc);
 protocol_crc_acc_buf(&crc, buf, sz);
 return crc;
}

int check_crc(uint32_t crc, uint8_t n){
	uint16_t*	crc_recv = (uint16_t*)(buffer + BUFFER_LEN - CRC_SIZE );
	//CDC_Transmit_FS((uint8_t*)buffer ,BUFFER_LEN);
	if (*crc_recv == crc) return 1;
	
	else return 0;
}

int check_uav_bim_state_pack(){
	#define BIM_STATE_PACK_LEN	4
	#define BIM_STATE_PACK_ID 	1
	#define BIM_STATE_PACK_VER	1
	if (!check_header(BIM_STATE_PACK_LEN, BIM_STATE_PACK_ID, BIM_STATE_PACK_VER)) return 0;
	uint16_t crc_comp = protocol_crc_calc(buffer, BUFFER_LEN);
	if (!check_crc(crc_comp,BIM_CONTROL_PACK_LEN)){ 
		return 0;
		//char ans[] = "its not uav bean state pack";
  //CDC_Transmit_FS((unsigned char*) ans ,strlen(ans));
	}
	struct uav_bim_state_t* state = (struct uav_bim_state_t*)(buffer + BUFFER_LEN - BIM_STATE_PACK_LEN - CRC_SIZE);
	pos_cmd[0] = state->left_ofs; 
	pos_cmd[1] = state->right_ofs;
	
	return 1;
}

int check_uav_bim_control_pack(){
	#define BIM_CONTROL_PACK_LEN	1
	#define BIM_CONTROL_PACK_ID 	3
	#define BIM_CONTROL_PACK_VER	1
	if (!check_header(BIM_CONTROL_PACK_LEN, BIM_CONTROL_PACK_ID, BIM_CONTROL_PACK_VER)) return 0;
	uint16_t crc_comp = protocol_crc_calc(buffer, BUFFER_LEN);
	if (!check_crc(crc_comp, BIM_CONTROL_PACK_LEN)) return 0;
	struct uav_bim_control_t* control = (struct uav_bim_control_t*)(buffer + BUFFER_LEN - BIM_STATE_PACK_LEN - CRC_SIZE);
	control_cmd = control->engine; 
	return 1;
}

int check_packet()
{
	if (check_uav_bim_state_pack()){
		
		char ans[] = "uav bim state pack";
  CDC_Transmit_FS((unsigned char*) ans ,strlen(ans));	
		return 1;
	}
	if (check_uav_bim_control_pack()){
		
		char ans[] = "uav bim control state pack";
  CDC_Transmit_FS((unsigned char*) ans ,strlen(ans));	
		return 1;
	}
		char ans[] = "no pack";
  CDC_Transmit_FS((unsigned char*) ans ,strlen(ans));	
	return 0;
}


enum
{
 UAV_BIM_ENGINE_OFF = 0,
 UAV_BIM_ENGINE_ON = 1,
 UAV_BIM_ENGINE_SAFEOFF = 2,
 UAV_BIM_ENGINE__LAST
};






int get_char(){
	
	if (uart_buffer_cur_len==0){
		return 0;
	}
	uint8_t i;
	// move buffer 1 char left
	for (i=0;i<BUFFER_LEN-1;i++) {
		buffer[i] = buffer[i+1];
	}
	// copy char from uart_buffer, reduce uart_len and move read_pos 
	uart_buffer_read++;
	if (uart_buffer_read >= UART_BUFFER_LEN) uart_buffer_read = 0; 
	buffer[BUFFER_LEN-1] = uart_buffer[uart_buffer_read];
	uart_buffer_cur_len--;
	
	//CDC_Transmit_FS((uint8_t*)uart_buffer ,uart_buffer_cur_len);
	//CDC_Transmit_FS((uint8_t*)uart_buffer ,BUFFER_LEN);
	//CDC_Transmit_FS((uint8_t*)buffer ,BUFFER_LEN);
	//check_packet();
	return 1;
}	

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char str_tx[21];
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
 sprintf(str_tx,"USB Transmit\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t counter; 
	counter = 0;	
	while (1)
  {
		// Blink--------------------------------------------
		counter += 1;
		if (counter>1000000){
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			counter = 0;
			//CDC_Transmit_FS((unsigned char*)str_tx, strlen(str_tx));
			//CDC_Transmit_FS((unsigned char*)uart_buffer, 32);
			
		}
		if(get_char()) {
			//CDC_Transmit_FS((uint8_t*)uart_buffer ,UART_BUFFER_LEN);
			HAL_Delay(100);
			CDC_Transmit_FS("\n" , 1);
			HAL_Delay(100);
			CDC_Transmit_FS((uint8_t*)buffer ,BUFFER_LEN);
			HAL_Delay(100);
			CDC_Transmit_FS("\n" ,1);
			HAL_Delay(100);
			CDC_Transmit_FS("\n" ,1);
			HAL_Delay(100);
			check_header(BIM_CONTROL_PACK_LEN, BIM_CONTROL_PACK_ID, BIM_CONTROL_PACK_VER );
			HAL_Delay(100);
			//check_packet();
			HAL_Delay(100);
			
		}
		//if(get_char()) CDC_Transmit_FS((uint8_t*)buffer ,uart_buffer_cur_len);

           
		
		
		//watchdog();
	
	//if (get_char()){
		
	//if (check_packet()){
			// got relevant packet. update watchdog timers
			//packet_wd_timeout = uptime + PACKET_WATCHDOG_TIMEOUT; 
		//	send_state();
		}
	}
		//if (get_char()){
		//	check_packet();
		//}
	

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

 // }
  /* USER CODE END 3 */

//}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
