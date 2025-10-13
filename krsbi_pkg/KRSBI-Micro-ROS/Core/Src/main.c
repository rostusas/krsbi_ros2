/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <krsbi_interfaces/msg/ball_position_based_on_camera.h>
#include <krsbi_interfaces/msg/wheel.h>
#include <krsbi_interfaces/msg/robot_mode.h>
#include <krsbi_interfaces/msg/kicker_module.h>
#include <math.h>
#include <pid.h>
#include <utility.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum states {
	WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED
} state;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis();} \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
  } while (0)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = { .name = "defaultTask",
		.stack_size = 3000 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* USER CODE BEGIN PV */
bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport,
		const uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf,
		size_t len, int timeout, uint8_t *err);

void* microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void* microros_reallocate(void *pointer, size_t size, void *state);
void* microros_zero_allocate(size_t number_of_elements, size_t size_of_element,
		void *state);

/*
 * Declare rcl object
 */
rclc_support_t support;
rcl_init_options_t init_options;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;

/*
 * TODO : Declare your
 * publisher & subscription objects below
 */
rcl_publisher_t pubWheel;
rcl_subscription_t subBallPositionBasedOnCamera;
rcl_subscription_t subRobotMode;
rcl_subscription_t subKickerModule;
rcl_subscription_t subPIDParam;

/*
 * TODO : Define your necessary Msg
 * that you want to work with below.
 */
krsbi_interfaces__msg__Wheel wheel_msg;
krsbi_interfaces__msg__BallPositionBasedOnCamera ballPositionBasedOnCamera_msg;
krsbi_interfaces__msg__RobotMode robotMode_msg;
krsbi_interfaces__msg__KickerModule kickerModule_msg;
std_msgs__msg__Float32MultiArray pidParam_msg;

PID_Controller pidAx, pidAy, pidW;
float ax, ay, w, errorJarak, errorSudut, errorX, errorY, force1, force2, force3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void kejar_bola_1(float *ax, float *w) {
	errorJarak = fabs(
			error_jarak_calculation(
					ballPositionBasedOnCamera_msg.position_set_point,
					ballPositionBasedOnCamera_msg.distance));
	errorSudut = error_sudut_calculation(
			ballPositionBasedOnCamera_msg.angle_set_point,
			ballPositionBasedOnCamera_msg.angle);
	*ax = PID_Update(&pidAx, ballPositionBasedOnCamera_msg.distance,
			errorJarak);
	*w = -PID_Update(&pidW, ballPositionBasedOnCamera_msg.angle, errorSudut);
}

void kejar_bola_2(float *ax, float *ay, float *w) {
	errorSudut = error_sudut_calculation(
			ballPositionBasedOnCamera_msg.angle_set_point,
			ballPositionBasedOnCamera_msg.angle);
	errorX = error_jarak_calculation(
			ballPositionBasedOnCamera_msg.position_x_set_point,
			ballPositionBasedOnCamera_msg.x_on_camera);
	errorY = error_jarak_calculation(
			ballPositionBasedOnCamera_msg.position_y_set_point,
			ballPositionBasedOnCamera_msg.y_on_camera);

	*ax =
			(PID_Update(&pidAx, ballPositionBasedOnCamera_msg.x_on_camera,
					errorY));
	*ay =
			(PID_Update(&pidAy, ballPositionBasedOnCamera_msg.y_on_camera,
					errorX));
	*w = PID_Update(&pidW, ballPositionBasedOnCamera_msg.angle, errorSudut);

}

void remap(float *force, float fromMinMax[2], float toMinMax[2]) {
	*force = ((*force - fromMinMax[0]) / (fromMinMax[1] - fromMinMax[0]))
			* (toMinMax[1] - toMinMax[0]) + toMinMax[0];
}

void motor_calculation(float ax, float ay, float w) {

	// Menghitung force untuk setiap motor
	force1 = ((-0.5773509529248335 * ax) + (-0.33333368867682667 * ay)
			+ (0.33333368867682667 * w));
	force2 = ((0.5773509529248335 * ax) + (-0.3333329779898402 * ay)
			+ (0.3333329779898402 * w));
	force3 = ((0 * ax) + (0.6666666666666669 * ay) + (0.33333333333333326 * w));

	// Remap jika force1 >= 0
	if (force1 >= 0) {
		float fromRange[2] = { 0, 0.91 };
		float toRange[2] = { 0, 255 };
		remap(&force1, fromRange, toRange);
		wheel_msg.left_wheel_cw = force1;
		wheel_msg.left_wheel_ccw = 0;
	} else if (force1 < 0) {
		force1 = fabs(force1);
		float fromRange[2] = { 0, 0.91 };
		float toRange[2] = { 0, 255 };
		remap(&force1, fromRange, toRange);
		wheel_msg.left_wheel_cw = 0;
		wheel_msg.left_wheel_ccw = force1;

	}
	if (force2 >= 0) {
		float fromRange[2] = { 0, 0.91 };
		float toRange[2] = { 0, 255 };
		remap(&force2, fromRange, toRange);
		wheel_msg.right_wheel_cw = force2;
		wheel_msg.right_wheel_ccw = 0;
	} else if (force2 < 0) {
		force2 = fabs(force2);
		float fromRange[2] = { 0, 0.91 };
		float toRange[2] = { 0, 255 };
		remap(&force2, fromRange, toRange);
		wheel_msg.right_wheel_cw = 0;
		wheel_msg.right_wheel_ccw = force2;

	}
	if (force3 >= 0) {
		float fromRange[2] = { 0, 0.91 };
		float toRange[2] = { 0, 255 };
		remap(&force3, fromRange, toRange);
		wheel_msg.bottom_wheel_cw = force3;
		wheel_msg.bottom_wheel_cw = 0;
	} else if (force3 < 0) {
		force3 = fabs(force3);
		float fromRange[2] = { 0, 0.91 };
		float toRange[2] = { 0, 255 };
		remap(&force3, fromRange, toRange);
		wheel_msg.bottom_wheel_cw = 0;
		wheel_msg.bottom_wheel_cw = force3;

	}
}

void ball_position_based_on_camera_callback(const void *msgin) {
	krsbi_interfaces__msg__BallPositionBasedOnCamera *ballPositionBasedOnCamera_msg =
			(krsbi_interfaces__msg__BallPositionBasedOnCamera*) msgin;

}

void robot_mode_callback(const void *msgin) {
	krsbi_interfaces__msg__RobotMode *robotMode_msg =
			(krsbi_interfaces__msg__RobotMode*) msgin;
}

void kicker_module_callback(const void *msgin) {
	krsbi_interfaces__msg__KickerModule *kickerModule_msg =
			(krsbi_interfaces__msg__KickerModule*) msgin;
}

void pidParam_callback(const void *msgin) {
	const std_msgs__msg__Float32MultiArray *pidParam_msg =
			(const std_msgs__msg__Float32MultiArray*) msgin;

	if (pidParam_msg->data.size >= 9) {
		pidAx.kp = pidParam_msg->data.data[0];
		pidAx.ki = pidParam_msg->data.data[1];
		pidAx.kd = pidParam_msg->data.data[2];

		pidAy.kp = pidParam_msg->data.data[3];
		pidAy.ki = pidParam_msg->data.data[4];
		pidAy.kd = pidParam_msg->data.data[5];

		pidW.kp = pidParam_msg->data.data[6];
		pidW.ki = pidParam_msg->data.data[7];
		pidW.kd = pidParam_msg->data.data[8];
	}
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
	(void) last_call_time;
	if (timer != NULL) {

		/*
		 TODO : Publish anything inside here
		 */
		if (strcasecmp(robotMode_msg.mode.data, "catchBall") == 0
				|| strcasecmp(robotMode_msg.mode.data, "catchBall2") == 0) {
			rcl_publish(&pubWheel, &wheel_msg, NULL);
		}

	}
}

bool create_entities() {
	/*
	 TODO : Define your
	 - ROS node name
	 - namespace
	 - ROS_DOMAIN_ID
	 */
	const char *node_name = "stm32_node";
	const char *ns = "";
	const int domain_id = 0;

	/*
	 * Initialize node
	 */
	allocator = rcl_get_default_allocator();
	init_options = rcl_get_zero_initialized_init_options();
	rcl_init_options_init(&init_options, allocator);
	rcl_init_options_set_domain_id(&init_options, domain_id);
	rclc_support_init_with_options(&support, 0, NULL, &init_options,
			&allocator);
	rclc_node_init_default(&node, node_name, ns, &support);

	/*
	 * TODO : Init your publisher and subscriber
	 */
	rclc_publisher_init(&pubWheel, &node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(krsbi_interfaces, msg, Wheel), "Wheel",
			&rmw_qos_profile_default);

	rclc_subscription_init(&subBallPositionBasedOnCamera, &node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(krsbi_interfaces, msg,
					BallPositionBasedOnCamera), "/BallPositionBasedOnCamera",
			&rmw_qos_profile_default);

	rclc_subscription_init(&subRobotMode, &node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(krsbi_interfaces, msg, RobotMode),
			"/RobotMode", &rmw_qos_profile_default);

	rclc_subscription_init(&subKickerModule, &node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(krsbi_interfaces, msg, KickerModule),
			"/KickerModule", &rmw_qos_profile_default);

	rclc_subscription_init(&subPIDParam, &node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
			"/pidParams", &rmw_qos_profile_default);

	/*
	 * Init timer_callback
	 * TODO : change timer_timeout
	 * 50ms : 20Hz
	 * 20ms : 50Hz
	 * 10ms : 100Hz
	 */
	const unsigned int timer_timeout = 10;
	rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout),
			timer_callback);

	/*
	 * Init Executor
	 * TODO : make sure the num_handles is correct
	 * num_handles = total_of_subscriber + timer
	 * publisher is not counted
	 *
	 * TODO : make sure the name of sub msg and callback are correct
	 */
	unsigned int num_handles = 5;
	executor = rclc_executor_get_zero_initialized_executor();
	rclc_executor_init(&executor, &support.context, num_handles, &allocator);
	rclc_executor_add_subscription(&executor, &subBallPositionBasedOnCamera,
			&ballPositionBasedOnCamera_msg,
			&ball_position_based_on_camera_callback, ON_NEW_DATA);
	rclc_executor_add_subscription(&executor, &subRobotMode, &robotMode_msg,
			&robot_mode_callback, ON_NEW_DATA);
	rclc_executor_add_subscription(&executor, &subKickerModule,
			&kickerModule_msg, &kicker_module_callback, ON_NEW_DATA);
	rclc_executor_add_subscription(&executor, &subPIDParam, &pidParam_msg,
			&pidParam_callback, ON_NEW_DATA);
	rclc_executor_add_timer(&executor, &timer);

	return true;
}

void destroy_entities() {
	rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
	(void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

	rcl_timer_fini(&timer);
	rclc_executor_fini(&executor);
	rcl_init_options_fini(&init_options);
	rcl_node_fini(&node);
	rclc_support_fini(&support);
	/*
	 * TODO : Make sue the name of publisher and subscriber are correct
	 */
	rcl_publisher_fini(&pubWheel, &node);
	rcl_subscription_fini(&subBallPositionBasedOnCamera, &node);
	rcl_subscription_fini(&subRobotMode, &node);
	rcl_subscription_fini(&subKickerModule, &node);
	rcl_subscription_fini(&subPIDParam, &node);
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

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
	MX_DMA_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	PID_Init(&pidAx, 15, 0.1, 0, -1000, 1000);
	PID_Init(&pidW, 6, 0, 0, -1000, 1000);
	PID_Init(&pidAy, 15, 0.1, 0, -1000, 1000);
	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,
			&defaultTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 12;
	RCC_OscInitStruct.PLL.PLLN = 96;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	/* DMA1_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, CHARGE_Pin | DISCHARGE_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : CHARGE_Pin DISCHARGE_Pin */
	GPIO_InitStruct.Pin = CHARGE_Pin | DISCHARGE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
	/* USER CODE BEGIN 5 */
	rmw_uros_set_custom_transport(
	true, (void*) &huart2, cubemx_transport_open, cubemx_transport_close,
			cubemx_transport_write, cubemx_transport_read);

	rcl_allocator_t freeRTOS_allocator =
			rcutils_get_zero_initialized_allocator();
	freeRTOS_allocator.allocate = microros_allocate;
	freeRTOS_allocator.deallocate = microros_deallocate;
	freeRTOS_allocator.reallocate = microros_reallocate;
	freeRTOS_allocator.zero_allocate = microros_zero_allocate;

	if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
		printf("Error on default allocators (line %d)\n", __LINE__);
	}

	kickerModule_msg.status.data = malloc(32);
	kickerModule_msg.status.size = 0;
	kickerModule_msg.status.capacity = 32;

	robotMode_msg.mode.data = malloc(32);
	robotMode_msg.mode.size = 0;
	robotMode_msg.mode.capacity = 32;

	static float pidParam_data[9] = {0}; // buffer statik agar tidak hilang dari stack
	pidParam_msg.data.size = 9;
	pidParam_msg.data.capacity = 9;
	pidParam_msg.data.data = pidParam_data;

	state = WAITING_AGENT;
	/* Infinite loop */
	for (;;) {
		switch (state) {
		case WAITING_AGENT:
			EXECUTE_EVERY_N_MS(500,
					state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
			break;
		case AGENT_AVAILABLE:
			state = (true == create_entities()) ?
					AGENT_CONNECTED : WAITING_AGENT;
			if (state == WAITING_AGENT) {
				destroy_entities();
			}
			;
			break;
		case AGENT_CONNECTED:
			EXECUTE_EVERY_N_MS(200,
					state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
			if (state == AGENT_CONNECTED) {
				rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
			}
			break;
		case AGENT_DISCONNECTED:
			destroy_entities();
			state = WAITING_AGENT;
			break;
		default:
			break;

		}
		/*
		 * TODO :
		 * Do anything else you want to do here,
		 * like read sensor data,
		 * calculate something, etc.
		 */
		if (strcasecmp(kickerModule_msg.status.data, "charging") == 0) {
			HAL_GPIO_WritePin(CHARGE_GPIO_Port, CHARGE_Pin, SET);
			HAL_GPIO_WritePin(DISCHARGE_GPIO_Port, DISCHARGE_Pin, RESET);
		} else if (strcasecmp(kickerModule_msg.status.data, "kick") == 0) {
			HAL_GPIO_WritePin(CHARGE_GPIO_Port, CHARGE_Pin, RESET);
			HAL_GPIO_WritePin(DISCHARGE_GPIO_Port, DISCHARGE_Pin, SET);
		} else {
			HAL_GPIO_WritePin(CHARGE_GPIO_Port, CHARGE_Pin, RESET);
			HAL_GPIO_WritePin(DISCHARGE_GPIO_Port, DISCHARGE_Pin, RESET);
		}

		if (strcasecmp(robotMode_msg.mode.data, "catchBall") == 0) {
			kejar_bola_1(&ax, &w);
		} else if (strcasecmp(robotMode_msg.mode.data, "catchBall2") == 0) {
			kejar_bola_2(&ax, &ay, &w);
		}

		map(&ax, -1000, 1000, -1, 1);
		map(&ay, -1000, 1000, -1, 1);
		map(&w, -1000, 1000, -1, 1);

		if (-0.13 <= ax && ax <= 0.13)
			ax = 0;
		if (0.13 <= w && w <= 0.13)
			w = 0;

		if (ax < -0.8)
			ax = -0.8;
		else if (ax > 0.6)
			ax = 0.6;

		if (ay < -0.8)
			ay = -0.8;
		else if (ay > 0.6)
			ay = 0.6;

		if (w < -0.8)
			w = -0.8;
		else if (w > 0.8)
			w = 0.8;

		motor_calculation(ax, ay, w);

		osDelay(1);
	}
	/* USER CODE END 5 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
