/*
 * rosserial.cpp
 *
 *  Created on: May 8, 2023
 *      Author: yeonwoo
 */

#include <stdio.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <cartbot/Speed.h>
#include <rosserial.h>

ros::NodeHandle nh;

//std_msgs::Int32 bat_msg;

//ros::Publisher bat_pub("bat/message", &bat_msg);

volatile int32_t tar_speed_L = 0;
volatile int32_t tar_speed_R = 0;
volatile bool mode_start = false;
//extern volatile uint32_t batPer;

volatile int32_t DT = 0;
volatile int32_t nowTime = 0;
volatile int32_t pastTime = 0;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART2)
		nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART2)
		nh.getHardware()->reset_rbuf();
}

void myCallback(const cartbot::Speed &msg)
{
	nowTime = HAL_GetTick();
	DT = nowTime - pastTime;
	tar_speed_L = msg.tar_rpm_l;
	tar_speed_R = msg.tar_rpm_r;
	pastTime = nowTime;
}

void userCallback(const std_msgs::Bool &msg)
{
	mode_start = msg.data;
}

ros::Subscriber <cartbot::Speed> rpm_sub("avoid/speed", myCallback);
ros::Subscriber <std_msgs::Bool> user_sub("user/message", userCallback);

void setup()
{
	nh.initNode();
	nh.subscribe(rpm_sub);
	nh.subscribe(user_sub);
	//nh.advertise(bat_pub);
}

void loop()
{
	/*nowTime = HAL_GetTick();
	if(nowTime - pastTime > 1000)
	{
		bat_msg.data = batPer;
		bat_pub.publish(&bat_msg);
		pastTime = nowTime;
	}*/
	nh.spinOnce();
}
