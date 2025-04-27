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
#include "config.h"
#include "ov7670.h"
#include <stdlib.h>
#include <stdbool.h>


/* USER CODE BEGIN PV */
#define PREAMBLE "\r\n!START!\r\n"
#define DELTA_PREAMBLE "\r\n!DELTA!\r\n"
#define SUFFIX "!END!\r\n"


uint16_t snapshot_buff[IMG_ROWS * IMG_COLS];
uint8_t old_snapshot_buff[IMG_ROWS * IMG_COLS];
unsigned int size;
uint8_t edge_image[IMG_ROWS * IMG_COLS];

uint8_t truncated_snapshot[IMG_ROWS * IMG_COLS /2];
uint8_t rle_snapshot[IMG_ROWS * IMG_COLS];

uint8_t tx_buff[sizeof(PREAMBLE) + 2 * IMG_ROWS * IMG_COLS + sizeof(SUFFIX)];
size_t tx_buff_len = 0;

// This is set in stm32f4xx_it.c in the DCMI_IRQHandler function.
uint8_t dma_flag = 0;

// Add function definitions for any other functions you add here.
void print_buf(void);

void print_test(void);
void print_img(void);
void truncate(uint8_t* snapshot, uint8_t* truncated, int size);
void print_truncated(void);
unsigned int rle(uint8_t* snapshot, uint8_t* result, int size);
void sobel_filter(uint8_t *image, uint8_t *output, int width, int height);
void highlight_edges(uint8_t *edge_image, int width, int height);
void draw_black_box_on_original(uint8_t *image, uint8_t *edge_image, int width, int height);
void overlay_edges_as_black(uint8_t *original, uint8_t *edges, int width, int height);
void draw_largest_edge_box_black(uint8_t *original, uint8_t *edge_image, int width, int height);




//edge detect
#include <stdint.h>

typedef struct {
    int x;
    int y;
} Point;

#define MAX_EDGE_POINTS 5000  // ???????????

void draw_detected_rectangle_from_edges(uint8_t *original, uint8_t *edge_image, int width, int height) {
    Point edge_points[MAX_EDGE_POINTS];
    int edge_point_count = 0;

    int min_x = width, max_x = 0, min_y = height, max_y = 0;

    // Step 1: ???????
    for (int y = 1; y < height - 1; y++) {
        for (int x = 1; x < width - 1; x++) {
            int idx = y * width + x;
            if (edge_image[idx] > 128) {  // Sobel ????
                if (edge_point_count < MAX_EDGE_POINTS) {
                    edge_points[edge_point_count++] = (Point){x, y};
                    // ??????
                    if (x < min_x) min_x = x;
                    if (x > max_x) max_x = x;
                    if (y < min_y) min_y = y;
                    if (y > max_y) max_y = y;
                }
            }
        }
    }

    // Step 2: ??????????,??????
    if (edge_point_count < 100) return;  // ????

    // Step 3: ??????????(4??)
    for (int x = min_x; x <= max_x; x++) {
        original[min_y * width + x] = 0;  // ??
        original[max_y * width + x] = 0;  // ??
    }
    for (int y = min_y; y <= max_y; y++) {
        original[y * width + min_x] = 0;  // ??
        original[y * width + max_x] = 0;  // ??
    }
}



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
	
	
  
  while (1)
  {	
		// part 3.3 RLE	
		
    while (HAL_UART_GetState(&huart3) != HAL_UART_STATE_READY) {}
		HAL_DCMI_Stop(&hdcmi);
		HAL_Delay(10);
		//ov7670_capture(snapshot_buff);
    uint8_t* ptr = (uint8_t*)snapshot_buff;
    for (int i = 0; i < (IMG_ROWS * IMG_COLS); i++) {
			old_snapshot_buff[i] = *(ptr + 2 * i + 1) & 0xF0;  // Keep only MSB
    }
		
		
		// edge detection
		sobel_filter(old_snapshot_buff, edge_image, IMG_COLS, IMG_ROWS);
		
		// highlight edge
		// 
		overlay_edges_as_black(old_snapshot_buff, edge_image, IMG_COLS, IMG_ROWS);
		//draw_detected_rectangle_from_edges(old_snapshot_buff, edge_image, IMG_COLS, IMG_ROWS);

		//size = rle(edge_image, rle_snapshot, IMG_ROWS * IMG_COLS + 1);
		size = rle(old_snapshot_buff, rle_snapshot, IMG_ROWS * IMG_COLS + 1);
		ov7670_capture(snapshot_buff);
		
		
		print_msg("\r\n!START!\r\n");
		while (HAL_UART_GetState(&huart3) != HAL_UART_STATE_READY) {}
			
    // Send the RLE encoded image
    //uart_send_bin(rle_snapshot, size);  // ? Use `size` instead of `size+1`
			
		uart_send_bin(rle_snapshot, size);

    while (HAL_UART_GetState(&huart3) != HAL_UART_STATE_READY) {}
		print_msg("!END!\r\n");
		while (HAL_UART_GetState(&huart3) != HAL_UART_STATE_READY) {}

    HAL_Delay(50);	

  }
}



//sobel edge detection
void sobel_filter(uint8_t *image, uint8_t *output, int width, int height) {
    int Gx, Gy, G;
    int sobel_x[3][3] = {
        {-1, 0, 1},
        {-2, 0, 2},
        {-1, 0, 1}
    };
    int sobel_y[3][3] = {
        {-1, -2, -1},
        {0, 0, 0},
        {1, 2, 1}
    };

    for (int y = 1; y < height - 1; y++) {
        for (int x = 1; x < width - 1; x++) {
            Gx = 0;
            Gy = 0;
            for (int ky = -1; ky <= 1; ky++) {
                for (int kx = -1; kx <= 1; kx++) {
                    Gx += image[(y + ky) * width + (x + kx)] * sobel_x[ky + 1][kx + 1];
                    Gy += image[(y + ky) * width + (x + kx)] * sobel_y[ky + 1][kx + 1];
                }
            }
            G = abs(Gx) + abs(Gy);
            output[y * width + x] = (G > 128) ? 255 : G;
        }
    }
		for (int i = 0; i < 10; i++){
			output[25046+i] = 128;
		}
}


//overlay
void overlay_edges_as_black(uint8_t *original, uint8_t *edges, int width, int height) {
    for (int i = 0; i < width * height; i++) {
        if (edges[i] > 128) {  // ???? Sobel ?????????
            original[i] = 0;   // ??????????????
        }
    }
}


// part 3.3
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

