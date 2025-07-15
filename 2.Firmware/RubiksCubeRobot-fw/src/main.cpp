#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include <iostream>
#include <sstream>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "nr_micro_shell.h"

// SDA 21
// SCL 22
// magnetic sensor instance - I2C
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(25, 33, 32, 12);

BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(14, 27, 26, 13);
// angle set point variable
float target_angle = 0;
float target_angle0 = 0; // 电机0的值
float target_angle1 = 0; // 电机1的值
bool bHassend;

// 限制最大速度，弧度/秒
float kVelocityLimit = 100;

// attractor angle variable
float attract_angle = 0;

// 产生断续阻尼的效果
//  distance between attraction points
float attractor_distance = 10 * _PI / 180.0; // dimp each 45 degrees
float findAttractor(float current_angle)
{
  return round(current_angle / attractor_distance) * attractor_distance;
}

void shell_help_cmd(char argc, char *argv)
{
  unsigned int i = 0;
  for (i = 0; nr_shell.static_cmd[i].fp != NULL; i++)
  {
    shell_printf("%s, %s\n", nr_shell.static_cmd[i].cmd, nr_shell.static_cmd[i].description);
  }
}

// 控制板的状态
void shell_stat_cmd(char argc, char *argv)
{
  printf("电机0位置：%f, 电机1位置：%f\n", sensor.getAngle(), sensor1.getAngle());
}

void shell_pos_cmd(char argc, char *argv)
{
  if (argc >= 2)
  {
    target_angle0 = atof(&(argv[(int)argv[1]]));
  }

  if (argc >= 3)
  {
    target_angle1 = atof(&(argv[(int)argv[2]]));
  }

  printf("电机0位置：%f => %f, 电机1位置：%f => %f\n", sensor.getAngle(), target_angle0, sensor1.getAngle(), target_angle1);
}

const static_cmd_st static_cmd[] = {
    {"help", shell_help_cmd, "Display available commands and descriptions"},
    {"stat", shell_stat_cmd, "Show system status"},
    {"pos", shell_pos_cmd, "Test position loop control with PID parameters"},
    {"\0", NULL, NULL}};

void uart_init()
{
  uart_config_t uart_cfg = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1};
  uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);
  uart_param_config(UART_NUM_0, &uart_cfg);
}

void shell_task(void *pvParameters)
{
  uint8_t c;
  while (1)
  {
    if (uart_read_bytes(UART_NUM_0, &c, 1, portMAX_DELAY) > 0)
    {
      shell(c); // 将字符传递给 nr_micro_shell
    }
  }
}

void setup(void)
{
  // initialise magnetic sensor hardware
  Wire.setClock(400000);
  sensor.init();

  // link the motor to the sensor
  motor.linkSensor(&sensor);
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);
  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // set motion control loop to be used
  motor.controller = MotionControlType::angle;
  // velocity PI controller parameters
  motor.PID_velocity.P = 0.08f; // 速度P值，这个值不能填太大，否则容易抖动
  motor.PID_velocity.I = 0.02;  // 这个值越大，响应速度会慢下来
  motor.PID_velocity.D = 0;
  motor.PID_velocity.output_ramp = 1500; // 调整这个值可以影响电机的加速和减速性能。较高的值会使电机加速和减速更快，但可能导致振动或电流峰值。
  // maximal voltage to be set to the motor
  motor.voltage_limit = 6; // 限制电压最大值，这个值一般为电源电压的一半
  // maximal velocity of the position control
  motor.velocity_limit = kVelocityLimit; // 限制最大速度，弧度/秒
  // velocity low pass filtering time constant
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01f; // 滤波,这可以滤除电机的噪声和高频振动，从而使速度控制更加稳定。
  // angle P controller
  motor.P_angle.P = 50; // 位置PID的P值

  // 新增一个磁编码器
  Wire1.setClock(400000);
  Wire1.begin(19, 23, (uint32_t)400000);
  sensor1.init(&Wire1);

  // 新增一个电机
  motor1.linkSensor(&sensor1);
  driver1.voltage_power_supply = 12;
  driver1.init();
  motor1.linkDriver(&driver1);
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.controller = MotionControlType::angle;
  motor1.PID_velocity.P = 0.08f; // 速度P值，这个值不能填太大，否则容易抖动
  motor1.PID_velocity.I = 0.02;  // 这个值越大，响应速度会慢下来
  motor1.PID_velocity.D = 0;
  motor1.PID_velocity.output_ramp = 1200; // 调整这个值可以影响电机的加速和减速性能。较高的值会使电机加速和减速更快，但可能导致振动或电流峰值。
  motor1.voltage_limit = 6;
  motor1.velocity_limit = kVelocityLimit; // 限制最大速度，弧度/秒
  motor1.LPF_velocity.Tf = 0.01f;
  motor1.P_angle.P = 50;

  // use monitoring with serial
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);
  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  // 新增电机初始化
  motor1.useMonitoring(Serial);
  motor1.init();
  motor1.initFOC();

  _delay(1000);

  uart_init();
  shell_init(); // 初始化 Shell
  xTaskCreate(shell_task, "shell_task", 2048 * 8, NULL, 5, NULL);
}

void loop()
{
  motor.loopFOC();            // 给上劲
  motor1.loopFOC();           // 给上劲
  motor.move(target_angle0);  // 电机0实时控制到目标角度
  motor1.move(target_angle1); // 电机1实时控制到目标角度
}