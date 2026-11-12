//
// Created by LEGION on 2021/10/4.
//
//This file contains the main function and the interrupt service routines (ISRs).
//The usage of each module follows three steps: initialization, data acquisition, and execution.
//Initialization and execution are performed within the timer interrupt routine.
//Serial (UART) data reception is handled within the UART interrupt routine.
//
//此文件为主函数和中断服务函数所在地
//每个元件的使用包含三个步骤，初始化、获取数据、执行
//初始化和执行在定时器中断函数进行
//获取串口数据在串口中断函数进行
/**
 * @file    Usermain.cpp
 * @brief   用户主程序：设备注册表、HAL 中断回调与应用 main()。
 * @details 所属模块：userCode/devices。采用「设备指针数组 + 三段式接口」框架：
 *          1) main() 完成 HAL/Clock/CubeMX 外设初始化，读取 Flash 标定参数，
 *             依次调用 device[] 中每个对象的 Init()，最后置 system_init_flag；
 *          2) HAL_TIM_PeriodElapsedCallback() 在 TIM1 的 300 Hz 更新中断中
 *             置 tick_300hz，主循环据此周期调用所有设备的 Handle()；
 *          3) HAL_UARTEx_RxEventCallback() 在 USART6 空闲中断中把 RxBuffer
 *             广播给所有设备的 Receive()，清空缓冲并重新开启 IT 接收；
 *          4) HAL_UART_TxCpltCallback() 把发送完成事件转发给
 *             UARTBaseLite<ID>::OnTxDone()，驱动队列中的下一帧。
 *          文件末尾 send_float()/send_int() 把数值格式化为文本并经
 *          UARTBaseLite<6> 的中断队列发回上位机。
 * @note    device[] 决定本版本实际启用的设备（用注释切换 C 板直驱 / 扩展板
 *          方案、看门狗、蜂鸣器等）；DEVICE_NUM 由数组长度自动计算。
 * @note    全局 RxBuffer 被所有设备共享，Receive() 中解析顺序需注意 strtok
 *          等函数会修改缓冲区；system_init_flag 未置位前定时器回调直接返回，
 *          避免初始化期间访问尚未就绪的设备。
 * @note    Robot_Version 需按实机硬件版本（V30~V40）修改，它决定舵机通道
 *          映射、水压计大气压基准等与版本相关的参数。
 */
#include "Usermain.h"
#include "IMU.h"
#include "Propeller.h"
#include "Servo.h"
#include "Sensor.h"
#include "Legacy.h"
#include "Watchdog.h"
#include "LED.h"
#include "Buzzer.h"
#include "gpio.h"
#include "UART_Base.h"


//Adjust according to the vehicle version: V30, V31, V32, V33, V40.
//根据潜器版本调整，V30，V31，V32，V33，V40
VERSION_E Robot_Version = V33;


// Statically instantiated objects.
// 静态实例化对象
// IMU imu;
static Servo servo;
static Servo_I2C servo_i2c;
static Propeller_I2C propeller_i2c;
static Sonar sonar;
static Watchdog watchdog;
static LED led;
static Buzzer buzzer;


// Array of device pointers for unified management and invocation of device operations.
// ------ TODO: Select the required devices.
// 设备指针数组，用于统一管理和调用设备操作
//------TODO:选用需要的设备
static Device *device[] = {
        &IMU::imu,                         //IMU 
        &PressureSensor::pressure_sensor,  //Pressure Sensor 水压计
        &propeller_i2c,                    //Propeller（PWM control via the PWM expansion board） 推进器（PWM扩展板控制）
        //&propeller,                      //Propeller（PWM control via the Robomaster_C board） 推进器（C板PWM控制）
				&servo_i2c,                        //Servo（PWM control via the PWM expansion board） 舵机（PWM扩展板控制）
				//&servo,                          //Servo（PWM control via the Robomaster_C board） 舵机（C板PWM控制）
        //&watchdog,                       //Watchdog 窗口看门狗
        &led,                              //LED
        //&buzzer                          //Buzzer 蜂鸣器
};

// Macro definition for the number of devices.
// 定义设备数量宏
#define DEVICE_NUM (sizeof(device) / sizeof(device[0]))

volatile bool system_init_flag = false;
volatile bool tick_300hz = false;
uint8_t RxBuffer[SERIAL_LENGTH_MAX] = {0};
volatile uint8_t key_raw_state = 1;
uint32_t key_last_stamp;
volatile int32_t time_start = 0, time_end = 0, time_interval = 0, cnt = 0;

// Timer interrupt service routine (ISR) for periodic device processing.
// 定时器中断服务函数, 用于周期性处理设备
/**
 * @name HAL 中断回调组
 * @details HAL_TIM_PeriodElapsedCallback：TIM1 300 Hz 更新中断中置
 *          tick_300hz（system_init_flag 未就绪时直接返回），主循环据此驱动
 *          Handle()；HAL_UARTEx_RxEventCallback：USART6 空闲帧到达时把
 *          RxBuffer 广播给所有设备 Receive()，清空缓冲并重启 IT 接收；
 *          HAL_UART_TxCpltCallback：借助 HANDLE_UART_TX 宏把发送完成事件
 *          分发到 UARTBaseLite<1>/<6> 的 OnTxDone()，继续发送队列下一帧。
 * @note  这些回调运行在中断上下文，须避免阻塞；RxBuffer 为全局共享，
 *        设备 Receive() 中的解析不得互相破坏尚未处理的帧内容。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(!system_init_flag) return;
    if(htim == &htim1){ //300Hz
        tick_300hz = true;
    }
}

// UART receive interrupt service routine (ISR) for handling device-specific UART reception tasks.
// 串口接收中断服务函数, 用于处理设备的串口接收任务
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t datasize)//HAL_UART_RxCpltCallback
{
    /* 函数内部流程步骤：
     * 步骤 1：判断中断来源是否为 USART6，否则直接退出。
     * 步骤 2：遍历 device[]，把全局 RxBuffer 交给每个设备的 Receive() 解析
     *         （原先按 "IM"/"PS"/"PR"/"SE"/"LE" 前缀分发的分支已注释）。
     * 步骤 3：std::memset 清空整个 RxBuffer，避免下一帧残留。
     * 步骤 4：HAL_UARTEx_ReceiveToIdle_IT() 重新开启空闲中断接收下一帧。
     */
    if(huart->Instance==USART6) {
        // if (datasize >= 2){
            // if (strncmp((char *)RxBuffer, "IM", 2) == 0)
            //     device[0]->Receive();
            // else if (strncmp((char *)RxBuffer, "PS", 2) == 0)
            //     device[1]->Receive();
            // else if (strncmp((char *)RxBuffer, "PR", 2) == 0)
            //     device[2]->Receive();
            // else if (strncmp((char *)RxBuffer, "SE", 2) == 0)
            //     device[3]->Receive();
            // else if (strncmp((char *)RxBuffer, "LE", 2) == 0)
            //     device[4]->Receive();
            for(int i=0;i<DEVICE_NUM;++i){
                device[i]->Receive();
            // }
        }
        
        std::memset(RxBuffer, 0, SERIAL_LENGTH_MAX);
        // Re-enable UART reception.
				// 重新开启接收
        HAL_UARTEx_ReceiveToIdle_IT(&huart6, RxBuffer, SERIAL_LENGTH_MAX);
    }
}

// UART transmit interrupt callback function.
// 串口发送中断回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    #define HANDLE_UART_TX(ID)                            \
        do {                                              \
            if (huart == uartHandleList[ID]) {            \
                UARTBaseLite<ID>::GetInstance().OnTxDone(); \
                return;                                   \
            }                                             \
        } while (0)

    HANDLE_UART_TX(1);
    HANDLE_UART_TX(6);

    #undef HANDLE_UART_TX
    // if(huart == &huart1) {
    //     UARTBaseLite<1>::GetInstance().OnTxDone();
    // }
    // else if(huart == &huart6){
    //     UARTBaseLite<6>::GetInstance().OnTxDone();
    // }
}

// Key/button interrupt (currently unused).
// 按键中断，暂时无用
// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//     if(system_init_flag == 0)return;
//     if (GPIO_Pin == GPIO_PIN_0){
//         key_raw_state = HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin);
//         uint32_t time_interval = HAL_GetTick() - key_last_stamp;
//         key_last_stamp = HAL_GetTick();
//         /*if (time_interval >= 50){
//             if (key_raw_state == 1){
//                 period = 5000;
//             }else{
//                 period = 500;
//             }
//         }*/
//     }
//     IMU::imu.ITHandle(GPIO_Pin);
// }

/**
  * @brief  The application entry point.
  * @retval int
  */
/**
 * @brief 应用入口：完成 MCU/外设/设备初始化后进入事件驱动主循环。
 * @return 理论上不返回（while(1)）。
 * @details 顺序：HAL_Init() -> SystemClock_Config() -> 各 MX_*_Init() ->
 *          启动 TIM5/TIM1/TIM8/TIM10/TIM6/TIM7 的时基与 PWM（带中断） ->
 *          读取 Flash 标定参数 -> 逐个 device[i]->Init() ->
 *          置 system_init_flag 并进入主循环；主循环只在 tick_300hz 为真时
 *          遍历 device[] 调用 Handle()，其余时间空转等待中断。
 * @note  MX_IWDG_Init() 被注释，硬件看门狗未启用（见 Watchdog.cpp 说明）；
 *        RxEventCallback 与 TxCpltCallback 共同承担串口收发，主循环不阻塞。
 */
// Entry point of the main function.
// 主函数入口
int main(void)
{
    /* 函数内部流程步骤：
     * 步骤 1：HAL_Init() 复位外设、初始化 Flash 接口与 SysTick。
     * 步骤 2：SystemClock_Config() 配置系统时钟树。
     * 步骤 3：依次调用各 MX_*_Init() 初始化 GPIO/DMA/定时器/ADC/串口/
     *         CAN/I2C/SPI/USB（IWDG 初始化被注释，硬件看门狗未启用）。
     * 步骤 4：启动 TIM5/TIM1/TIM8/TIM10/TIM6/TIM7 的时基与 PWM 通道（带中断）。
     * 步骤 5：bsp_flash_read(&flashData) 读取 Flash 中的水压计零偏等标定参数。
     * 步骤 6：遍历 device[] 调用每个设备对象的 Init()，随后置
     *         system_init_flag = true 并记录起始时间戳。
     * 步骤 7：进入 while(1)：仅当 tick_300hz 为真时清标志并遍历
     *         device[]->Handle()，其余时间空转等待中断，
     *         串口收发与定时节拍完全由 HAL 回调驱动。
     */
    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM5_Init();
    MX_TIM1_Init();
    MX_TIM4_Init();
    MX_TIM6_Init();
    MX_TIM7_Init();
    MX_TIM10_Init();
    MX_TIM8_Init();
    MX_ADC1_Init();
    MX_ADC3_Init();
    MX_USART1_UART_Init();
    MX_USART6_UART_Init();
    MX_USART3_UART_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_I2C2_Init();
    MX_I2C3_Init();
    MX_SPI1_Init();
    MX_SPI2_Init();
    //MX_IWDG_Init();
    MX_USB_DEVICE_Init();

    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);


    HAL_TIM_Base_Start_IT(&htim8);
    HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_3);
    //TODO adc校准？
    //RemoteControl::init();
    bsp_flash_read(&flashData);
    HAL_TIM_Base_Start_IT(&htim10);
    HAL_TIM_PWM_Start_IT(&htim10, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);

		// Initialize all devices.
    // 初始化所有设备
    for(int i = 0; i < DEVICE_NUM; ++i){
        device[i] -> Init();
    }
		
		// Set the initialization-complete flag.
    // 设置初始化完成标志
    system_init_flag = true;
    time_start = HAL_GetTick();

    while (1){
        // time_start = HAL_GetTick();
        if(tick_300hz){
            tick_300hz = false;
            static int a = 0;
            a++;
            for(int i = 0 ; i < DEVICE_NUM ; ++i){
                device[i]->Handle();
//                if (a==150&&i==4)
//                {
//                    time_end = HAL_GetTick();
//                    time_interval = time_end - time_start;
//                    send_int(time_interval, 1);
//                    a = 0;
//                    time_start = HAL_GetTick();
//                }
            }
            
        }
    }
}

void send_float(float value, uint8_t decimalPlaces, bool endSign){
    /* 函数内部流程步骤：
     * 步骤 1：准备静态 20 字节文本缓冲、长度 len 与负数标志。
     * 步骤 2：value < 0 时取绝对值并在缓冲首字节写 '-'，len 置 1。
     * 步骤 3：sprintf 把整数部分写入 buffer_float+len，并把返回长度累加到 len。
     * 步骤 4：decimalPlaces > 0 时先写 '.'，再逐位把小数乘 10 取整数位
     *         (digit + '0') 写入缓冲，共 decimalPlaces 位。
     * 步骤 5：按 endSign 追加 '\n' 或 ','，再写字符串结束符 '\0'。
     * 步骤 6：调用 UARTBaseLite<6>::GetInstance().Transmit() 把文本帧
     *         放入串口发送队列（非阻塞，由中断逐帧发出）。
     */
    bool isNegative = 0;
    uint8_t len = 0;
    static char buffer_float[20] = {0};
		
		// Handle negative values.
    // 处理负数
    if(value < 0){
        isNegative = 1;
        value = -value;
    }
    if(isNegative) {
        buffer_float[0] = '-';
        len = 1;
    }

		// Convert the integer part.
    // 转换整数部分
    int32_t intPart = (int)value;
    len += sprintf(buffer_float+len, "%d", intPart);
		
		// Process the fractional part.
    // 处理小数部分
    if (decimalPlaces > 0) {
        buffer_float[len++] = '.';

        float fraction = value - (float)intPart;
        for (uint8_t i = 0; i < decimalPlaces; i++) {
            fraction *= 10;
            int digit = (int)fraction;
            buffer_float[len++] = digit + '0';
            fraction -= digit;
        }
    }

		// If this is the final element, append a newline; otherwise, append a comma.
    // 若结束则换行，否则是逗号
    if(endSign)
        buffer_float[len++] = '\n'; // Null terminator of the string 字符串结束符
    else
        buffer_float[len++] = ',';

    buffer_float[len] = '\0';

    // HAL_UART_Transmit(&huart6, (uint8_t *)buffer, len, 0x00ff);
    UARTBaseLite<6>::GetInstance().Transmit(reinterpret_cast<uint8_t *>(buffer_float),len);
}

void send_int(int32_t value, bool endSign){
    /* 函数内部流程步骤：
     * 步骤 1：准备静态 20 字节缓冲、长度 len 与负数标志。
     * 步骤 2：value < 0 时取绝对值并写 '-'，len 置 1。
     * 步骤 3：sprintf 把整数写成十进制文本，长度累加到 len。
     * 步骤 4：按 endSign 追加 '\n' 或 ','，写 '\0' 结束字符串。
     * 步骤 5：交给 UARTBaseLite<6> 的发送队列异步发出。
     */
    static char buffer_int[20] = {0};
    uint8_t len = 0;
    bool isNegative = 0;

		// Handle negative values.
    // 处理负数
    if(value < 0){
        isNegative = 1;
        value = -value;
    }
    if(isNegative) {
        buffer_int[0] = '-';
        len = 1;
    }

    len += sprintf(buffer_int+len, "%d", value);

    if(endSign)
        buffer_int[len++] = '\n';
    else
        buffer_int[len++] = ',';

    buffer_int[len] = '\0';
    // HAL_UART_Transmit(&huart6, (uint8_t *)buffer, len, 0x00ff);
    UARTBaseLite<6>::GetInstance().Transmit(reinterpret_cast<uint8_t *>(buffer_int),len);
}