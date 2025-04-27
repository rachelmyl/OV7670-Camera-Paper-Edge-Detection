 /* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "config.h"
#include "ov7670.h"

/* USER CODE BEGIN PV */
#define PREAMBLE "\r\n!START!\r\n"
#define DELTA_PREAMBLE "\r\n!DELTA!\r\n"
#define SUFFIX "!END!\r\n"

uint16_t snapshot_buff[IMG_ROWS * IMG_COLS];
uint8_t old_snapshot_buff[IMG_ROWS * IMG_COLS];
uint8_t rle_snapshot[IMG_ROWS * IMG_COLS];
uint8_t prev_frame[IMG_ROWS * IMG_COLS];
uint8_t edge_map[IMG_ROWS * IMG_COLS];

uint8_t tx_buff[sizeof(PREAMBLE) + 2 * IMG_ROWS * IMG_COLS + sizeof(SUFFIX)];
size_t tx_buff_len = 0;

unsigned int size;

// This is set in stm32f4xx_it.c in the DCMI_IRQHandler function.
uint8_t dma_flag = 0;

// Add function definitions for any other functions you add here.
void print_buf(void);

void print_img(unsigned int size);
void rle_capture(void);
unsigned int rle_snap(uint8_t* snapshot);
unsigned int rle(uint8_t* snapshot, uint8_t* result, int size);
void sobel_edge_detection(uint8_t *input, uint8_t *output, int width, int height);

	

int main(void)
{
  /* Reset of all peripherals */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DCMI_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  
  char msg[100];
  
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  
  ov7670_init();
  HAL_Delay(100);
  ov7670_capture(snapshot_buff);
  
  // Add variables you need here.
	bool stop = 0;
	unsigned int snapshot_size;
	int count = 0;
	
  while (1)
  {
		if (dma_flag == 1){
			dma_flag = 0;
			
			if (!stop){
				HAL_DCMI_Stop(&hdcmi);
				rle_capture();
				snapshot_size = size;
				print_img(snapshot_size);
				stop = 1;
			}
			else { stop = 0; }
			HAL_Delay(100);
		}
		
		if (stop){
			// print snapshot_buff
			print_img(snapshot_size);
		}
		else{
			// print rle_snapshot
			rle_capture();
			print_img(size);
		}
  }
}

void print_img(unsigned int size){
	print_msg("\r\n!START!\r\n");
	
	while (HAL_UART_GetState(&huart3) != HAL_UART_STATE_READY) {}
		
  // Send the RLE encoded image
  uart_send_bin(rle_snapshot, size);  // ? Use `size` instead of `size+1`

  while (HAL_UART_GetState(&huart3) != HAL_UART_STATE_READY) {}
	print_msg("!END!\r\n");
	while (HAL_UART_GetState(&huart3) != HAL_UART_STATE_READY) {}

  HAL_Delay(50);
}

void rle_capture(){
	
	while (HAL_UART_GetState(&huart3) != HAL_UART_STATE_READY) {}
	HAL_DCMI_Stop(&hdcmi);

  uint8_t* ptr = (uint8_t*)snapshot_buff;
  for (int i = 0; i < (IMG_ROWS * IMG_COLS); i++) {
		old_snapshot_buff[i] = *(ptr + 2*i + 1) & 0xF0;  // Keep only MSB
  }
	sobel_edge_detection(old_snapshot_buff, edge_map, IMG_COLS, IMG_ROWS);
	size = rle(edge_map, rle_snapshot, IMG_ROWS * IMG_COLS + 1);

  ov7670_capture(snapshot_buff);
}

unsigned int rle(uint8_t* snapshot, uint8_t* result, int size){
	int count = 1;
	int j = 0;
	uint8_t prev = snapshot[0] & 0xF0;
	for(int i = 1; i < size; i++){
		uint8_t current = snapshot[i] & 0xF0;
		
		if(current == prev){
			count += 1;
		}
		else{
			uint8_t first = prev;
			uint8_t second = count & 0x0F;
			result[j] = first|second;
			j++;
			count = 1;
		}
		if(count > 15){
			uint8_t first = prev;
			uint8_t second = (count-1) & 0x0F;
			result[j] = first|second;
			j++;
			count = 1;
		}
		prev = current;
	}
	uint8_t first = snapshot[size-1] & 0xF0;
	uint8_t second = count & 0x0F;
	result[j] = first|second;
	
	return j;
}

void sobel_edge_detection(uint8_t *input, uint8_t *output, int width, int height) {
    int Gx, Gy;

    // Sobel kernels
    int kernel_x[3][3] = {{-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1}};
    int kernel_y[3][3] = {{-1, -2, -1}, {0, 0, 0}, {1, 2, 1}};

    for (int i = 1; i < height - 1; i++) {
        for (int j = 1; j < width - 1; j++) {
            Gx = 0;
            Gy = 0;

            // Apply Sobel kernels
            for (int x = -1; x <= 1; x++) {
                for (int y = -1; y <= 1; y++) {
                    Gx += input[(i + x) * width + (j + y)] * kernel_x[x + 1][y + 1];
                    Gy += input[(i + x) * width + (j + y)] * kernel_y[x + 1][y + 1];
                }
            }

            // Calculate gradient magnitude (approximation)
            int magnitude = abs(Gx) + abs(Gy);

            // Apply threshold
            // output[i * width + j] = (magnitude > 128) ? 255 : 0;
						if (magnitude > 170)
							output[i * width + j] = 255;
						else if (magnitude > 120)
							output[i * width + j] = 128;
						else
							output[i * width + j] = 0;
        }
    }
		for (int i = 0; i < 13; i++){
			output[25043+i] = 128;
		}
}


void print_buf() {
  
  // Create a new buffer from the snapshot_buffer than the DCMI copied the 16-bit pixel values into.
  uint8_t *buffer = (uint8_t *) snapshot_buff;
  
  // Add the START preamble message to the start of the buffer for the serial-monitor program. 
  for (int i = 0; i < sizeof(PREAMBLE); i++) {
    tx_buff[i] = PREAMBLE[i];
  }
  
  // Write code to copy every other byte from the main frame buffer to 
  // our temporary buffer (this converts the image to grey scale)
  
  
  
	
  // Load the END suffix message to the end of the message.
  for (int i = 0; i < sizeof(SUFFIX); i++) {
    tx_buff[tx_buff_len++] = SUFFIX[i];
  }
  
  // Once the data is copied into the buffer, call the function to send it via UART. 
  uart_send_bin(tx_buff, sizeof(PREAMBLE) + IMG_COLS * IMG_ROWS);
}

