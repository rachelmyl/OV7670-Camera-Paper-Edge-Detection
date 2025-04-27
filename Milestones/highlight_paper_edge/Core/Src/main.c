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
#include <math.h>
#include <stdbool.h>

/* USER CODE BEGIN PV */
#define PREAMBLE "\r\n!START!\r\n"
#define DELTA_PREAMBLE "\r\n!DELTA!\r\n"
#define SUFFIX "!END!\r\n"

#define MAX_CORNERS 250
#define EDGE_THRESHOLD 128

uint16_t snapshot_buff[IMG_ROWS * IMG_COLS];
uint8_t old_snapshot_buff[IMG_ROWS * IMG_COLS];
uint8_t rle_snapshot[IMG_ROWS * IMG_COLS + 2000];
uint8_t prev_frame[IMG_ROWS * IMG_COLS];
uint8_t edge_map[IMG_ROWS * IMG_COLS];
uint8_t perspective_map[IMG_ROWS * IMG_COLS];

typedef struct {
    int x;
    int y;
} Point;

Point paper_corners[4];

uint8_t tx_buff[sizeof(PREAMBLE) + 2 * IMG_ROWS * IMG_COLS + sizeof(SUFFIX)];
size_t tx_buff_len = 0;

unsigned int size;

// This is set in stm32f4xx_it.c in the DCMI_IRQHandler function.
uint8_t dma_flag = 0;

// Add function definitions for any other functions you add here.
void print_buf(void);

// print functions
void print_img(unsigned int size);
void rle_capture(void);
unsigned int rle_snap(uint8_t* snapshot);
unsigned int rle(uint8_t* snapshot, uint8_t* result, int size);

// edge detection functions
void sobel_edge_detection(uint8_t *input, uint8_t *output, int width, int height);
void highlight_page_edges(uint8_t *input, uint8_t *output, int width, int height);
void draw_line(uint8_t *image, int x1, int y1, int x2, int y2, int width);
bool is_corner(uint8_t *edges, int x, int y, int width, int height);
bool is_largest_quadrilateral(int x, int y, int *max_area, int corners[4][2]);
void overlay_edges_as_white(uint8_t *original, uint8_t *edges, int width, int height);
float quadrilateral_area(Point points[4]);
void overlay_paper_edges(uint8_t *original, uint8_t *edges, int width, int height);

// flatten perspective functions
void perspective_transform(uint8_t *input, uint8_t *output, int width, int height, Point corners[4]);
void calculate_homography(Point src[4], Point dst[4], int32_t H[9]);


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
			// print flattened version
			print_img(snapshot_size);
		}
		else{ 
			// print rle_snapshot
			rle_capture();
			print_img(size);
		}
  }
}

// Helper functions

// Edge detection

void overlay_edges_as_white(uint8_t *original, uint8_t *edges, int width, int height) {
    for (int i = 0; i < width * height; i++) {
        if (edges[i] > 128) {  
            original[i] = 255;   
        }
    }
}

// Check if a point is a corner (simplified version)
bool is_corner(uint8_t *edges, int x, int y, int width, int height) {
    // Simple check - look for L-shaped patterns in the edges
    int count = 0;
    
    // Check 8 surrounding pixels
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0) continue;
            if (x+dx >= 0 && x+dx < width && y+dy >= 0 && y+dy < height) {
                if (edges[(y+dy)*width + (x+dx)] > 128) {
                    count++;
                }
            }
        }
    }
    
    // Corners typically have >3 edge pixels nearby
    return (count == 8);
}


void draw_line(uint8_t *image, int x1, int y1, int x2, int y2, int width) {
		// Draw a line between two points (Bresenham's algorithm)
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;
    
    while (1) {
        // Draw the pixel
        if (x1 >= 0 && x1 < width && y1 >= 0 && y1 < IMG_ROWS) {
            image[y1 * width + x1] = 0; // Black
        }
        
        if (x1 == x2 && y1 == y2) break;
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }
}

// Calculate the area of a quadrilateral defined by 4 points
float quadrilateral_area(Point points[4]) {
    // Using shoelace formula for quadrilateral area
    float area = 0.0f;
    for (int i = 0; i < 4; i++) {
        int j = (i + 1) % 4;
        area += (points[i].x * points[j].y) - (points[j].x * points[i].y);
    }
    return fabs(area) / 2.0f;
}



// Helper function for convex hull
static int cross_product(const Point *o, const Point *a, const Point *b) {
    return (a->x - o->x) * (b->y - o->y) - (a->y - o->y) * (b->x - o->x);
}

// Comparison function for sorting points
static int compare_points(const void *a, const void *b) {
    const Point *p1 = (const Point *)a;
    const Point *p2 = (const Point *)b;
    if (p1->x != p2->x) return p1->x - p2->x;
    return p1->y - p2->y;
}

// Andrew's monotone chain convex hull algorithm
static int convex_hull(Point *points, int n, Point *hull) {
    if (n < 3) return n;
    
    qsort(points, n, sizeof(Point), compare_points);
    
    int k = 0;
    
    // Build lower hull
    for (int i = 0; i < n; i++) {
        while (k >= 2 && cross_product(&hull[k-2], &hull[k-1], &points[i]) <= 0) k--;
        hull[k++] = points[i];
    }
    
    // Build upper hull
    for (int i = n-2, t = k+1; i >= 0; i--) {
        while (k >= t && cross_product(&hull[k-2], &hull[k-1], &points[i]) <= 0) k--;
        hull[k++] = points[i];
    }
    
    return k-1; // Remove duplicate last point
}

bool find_largest_quadrilateral(uint8_t *edges, int width, int height, Point corners[4]) {
    Point potential_corners[MAX_CORNERS];
    int corner_count = 0;
    
    // Find potential corners (same as before)
    for (int y = 20; y < height - 20; y += 2) {
        for (int x = 20; x < width - 20; x += 2) {
            if (edges[y * width + x] > EDGE_THRESHOLD && 
                is_corner(edges, x, y, width, height)) {
                potential_corners[corner_count].x = x;
                potential_corners[corner_count].y = y;
                if (++corner_count >= MAX_CORNERS) break;
            }
        }
        if (corner_count >= MAX_CORNERS) break;
    }
    
    if (corner_count < 4) return false;
    
    Point hull[MAX_CORNERS];
    int hull_size = convex_hull(potential_corners, corner_count, hull);
    
    // If convex hull has exactly 4 points, use them
    if (hull_size == 4) {
        memcpy(corners, hull, 4 * sizeof(Point));
        return true;
    }
    
    // Otherwise find largest quadrilateral from convex hull points
    float max_area = 0;
    bool found = false;
    
    // Rotating calipers approach for convex polygons
    if (hull_size > 4) {
        for (int i = 0; i < hull_size; i++) {
            for (int j = (i+1) % hull_size; j != i; j = (j+1) % hull_size) {
                for (int k = (j+1) % hull_size; k != i; k = (k+1) % hull_size) {
                    for (int l = (k+1) % hull_size; l != i; l = (l+1) % hull_size) {
                        if (j == k || k == l || l == i) continue;
                        
                        Point quad[4] = {hull[i], hull[j], hull[k], hull[l]};
                        float area = quadrilateral_area(quad);
                        if (area > max_area) {
                            max_area = area;
                            memcpy(corners, quad, sizeof(quad));
                            found = true;
                        }
                    }
                }
            }
        }
    }
    
    return found;
}

// Update your overlay function to highlight the paper edges
void overlay_paper_edges(uint8_t *original, uint8_t *edges, int width, int height) {
    // Point paper_corners[4];
    if (!find_largest_quadrilateral(edges, width, height, paper_corners)){
			return;
		}
    
    // Draw the quadrilateral edges
    for (int i = 0; i < 4; i++) {
        int j = (i + 1) % 4;
        draw_line(original, paper_corners[i].x, paper_corners[i].y, 
                 paper_corners[j].x, paper_corners[j].y, width);
    }
    
    // Also highlight the corners
    for (int i = 0; i < 4; i++) {
        int x = paper_corners[i].x;
        int y = paper_corners[i].y;
        for (int dy = -2; dy <= 2; dy++) {
            for (int dx = -2; dx <= 2; dx++) {
                if (x+dx >= 0 && x+dx < width && y+dy >= 0 && y+dy < height) {
                    original[(y+dy)*width + (x+dx)] = 0; // Black
                }
            }
        }
    }
}


// Print img functions

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
	
	// apply sobel filter
	sobel_edge_detection(old_snapshot_buff, edge_map, IMG_COLS, IMG_ROWS);
	// highlight_page_edges(old_snapshot_buff, edge_map, IMG_COLS, IMG_ROWS);
	// overlay_edges_as_white(old_snapshot_buff, edge_map, IMG_COLS, IMG_ROWS);
	overlay_paper_edges(old_snapshot_buff, edge_map, IMG_COLS, IMG_ROWS);
	size = rle(old_snapshot_buff, rle_snapshot, IMG_ROWS * IMG_COLS + 1);

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
            output[i * width + j] = (magnitude > 150) ? 255 : 0;
						/*
						if (magnitude > 170)
							output[i * width + j] = 255;
						else if (magnitude > 120)
							output[i * width + j] = 128;
						else
							output[i * width + j] = 0;
						*/
        }
    }
		/*
		for (int i = 0; i < 13; i++){
			output[25043+i] = 0;
		}
		*/
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

