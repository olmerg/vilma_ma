/**
 *
	 @brief This is the library with all the topics generated (published) and that receive (suscribers) the microautobox
        based on the code of the plug-in DRCVehicleROSPlugin of the drcsim 2.7 of Open Source Robotics Foundation

 * @author olmer Garcia olmerg@gmail.com
 * Copyright 2013 Olmer Garcia Bedoya
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include "vilma_MA_ROS.h"
#include <math.h>
#include <stdlib.h>
#include <sched.h>

vilma_MA_ROS::vilma_MA_ROS():clientMA()
{
  // ros stuff
  this->rosNode = new ros::NodeHandle("");
  int port, ma_port, timeout;
  String ma_ip;
	this->rosNode->param("microautobox/udp_port", port, 5001);
	this->rosNode->param("microautobox/ma_port", ma_port, 5001);
	this->rosNode->param("microautobox/ma_ip", ma_ip, "192.168.140.3");
	this->rosNode->param("microautobox/timeout", timeout, 3);
	this->rosNode->param("microautobox/Ts", this->rosPublishPeriod, 0.01);

if(!clientMA.configure(port,ma_port,ma_ip,boost::posix_time::millisec(3)))
    ROS_ERROR("the udp port in the pc is not accessible");
	
 // this->rosPublishPeriod=0.1;
  //this->lastRosPublishTime = ros::Time(0.0);




    ros::SubscribeOptions hand_wheel_cmd_so =
      ros::SubscribeOptions::create<std_msgs::Float64>(
      "vilma_MA/hand_wheel/cmd", 10,
      boost::bind(static_cast< void (vilma_MA_ROS::*)
        (const std_msgs::Float64::ConstPtr&) >(
          &vilma_MA_ROS::SetHandWheelState), this, _1),
      ros::VoidPtr(),  NULL);//&this->queue);
    this->subHandWheelCmd = this->rosNode->subscribe(hand_wheel_cmd_so);
  
    ros::SubscribeOptions hand_brake_cmd_so =
      ros::SubscribeOptions::create< std_msgs::Float64 >(
      "vilma_MA/hand_brake/cmd", 10,
      boost::bind(static_cast< void (vilma_MA_ROS::*)
        (const std_msgs::Float64::ConstPtr&) >(
          &vilma_MA_ROS::SetHandBrakePercent), this, _1),
      ros::VoidPtr(),  NULL);//&this->queue);
    this->subHandBrakeCmd = this->rosNode->subscribe(hand_brake_cmd_so);
  
    ros::SubscribeOptions gas_pedal_cmd_so =
      ros::SubscribeOptions::create< std_msgs::Float64 >(
      "vilma_MA/gas_pedal/cmd", 10,
      boost::bind(static_cast< void (vilma_MA_ROS::*)
        (const std_msgs::Float64::ConstPtr&) >(
          &vilma_MA_ROS::SetGasPedalPercent), this, _1),
      ros::VoidPtr(),  NULL);//&this->queue);
    this->subGasPedalCmd = this->rosNode->subscribe(gas_pedal_cmd_so);
  
    ros::SubscribeOptions brake_pedal_cmd_so =
      ros::SubscribeOptions::create< std_msgs::Float64 >(
      "vilma_MA/brake_pedal/cmd", 10,
      boost::bind(static_cast< void (vilma_MA_ROS::*)
        (const std_msgs::Float64::ConstPtr&) >(
          &vilma_MA_ROS::SetBrakePedalPercent), this, _1),
      ros::VoidPtr(), NULL);//&this->queue);
    this->subBrakePedalCmd = this->rosNode->subscribe(brake_pedal_cmd_so);
  
  
    ros::SubscribeOptions direction_cmd_so =
      ros::SubscribeOptions::create< std_msgs::Int8 >(
      "vilma_MA/direction/cmd", 10,
      boost::bind(static_cast< void (vilma_MA_ROS::*)
        (const std_msgs::Int8::ConstPtr&) >(
          &vilma_MA_ROS::SetDirectionState), this, _1),
      ros::VoidPtr(), NULL);// &this->queue);
    this->subDirectionCmd = this->rosNode->subscribe(direction_cmd_so);

    this->pubHandWheelState = this->rosNode->advertise<std_msgs::Float64>(
      "vilma_MA/hand_wheel/state", 10);
    this->pubHandBrakeState = this->rosNode->advertise<std_msgs::Float64>(
      "vilma_MA/hand_brake/state", 10);
    this->pubGasPedalState = this->rosNode->advertise<std_msgs::Float64>(
      "vilma_MA/gas_pedal/state", 10);
    this->pubBrakePedalState = this->rosNode->advertise<std_msgs::Float64>(
      "vilma_MA/brake_pedal/state", 10);
    this->pubDirectionState = this->rosNode->advertise<std_msgs::Int8>(
      "vilma_MA/direction/state", 10);
	lastcall=ros::Time::now().toSec();
	ros::WallTimerOptions timer_op=ros::WallTimerOptions(ros::WallDuration(this->rosPublishPeriod),boost::bind(&vilma_MA_ROS::RosPublishStates,this,_1),&this->queue);
	timer=this->rosNode->createWallTimer(timer_op);

    // ros callback queue for processing subscription
    this->callbackQueueThread = boost::thread(
      boost::bind(&vilma_MA_ROS::QueueThread, this));
	struct sched_param param;
	param.sched_priority = 30;
	pthread_t threadID = (pthread_t) callbackQueueThread.native_handle();
	pthread_setschedparam( threadID, SCHED_FIFO,&param);


    //timer=this->rosNode->createTimer(ros::Duration(this->rosPublishPeriod),&vilma_MA_ROS::RosPublishStates,this);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
vilma_MA_ROS::~vilma_MA_ROS()
{
 
  this->rosNode->shutdown();
  this->queue.clear();
  this->queue.disable();
  this->callbackQueueThread.join();
  delete this->rosNode;
}

////////////////////////////////////////////////////////////////////////////////
void vilma_MA_ROS::SetDirectionState(
  const std_msgs::Int8::ConstPtr &_msg)
{
  /*if (_msg->data == 0)
    this->DRCVehiclePlugin::SetDirectionState(NEUTRAL);
  else if (_msg->data == 1)
    this->DRCVehiclePlugin::SetDirectionState(FORWARD);
  else if (_msg->data == -1)
    this->DRCVehiclePlugin::SetDirectionState(REVERSE);
  else
    ROS_ERROR("Invalid Direction State: %d, expected -1, 0, or 1\n",
      static_cast<int16_t>(_msg->data));*/
  ROS_INFO("gearstick information %d",static_cast<int16_t>(_msg->data));

 // this->UpdateFNRSwitchTime();
}

////////////////////////////////////////////////////////////////////////////////
void vilma_MA_ROS::SetHandBrakePercent(const std_msgs::Float64::ConstPtr
    &_msg)
{
  double min, max, percent, cmd;
  percent = clamp(static_cast<double>(_msg->data), 0.0, 1.0);
/*  DRCVehiclePlugin::GetHandBrakeLimits(min, max);
  cmd = min + percent * (max - min);
  DRCVehiclePlugin::SetHandBrakeState(cmd);*/
  ROS_INFO("handbrake information %f",percent);
}

////////////////////////////////////////////////////////////////////////////////
void vilma_MA_ROS::SetHandWheelState(const std_msgs::Float64::ConstPtr
    &_msg)
{
//  DRCVehiclePlugin::SetHandWheelState(static_cast<double>(_msg->data));
  ROS_INFO("steering information %f",static_cast<double>(_msg->data));
}

////////////////////////////////////////////////////////////////////////////////
void vilma_MA_ROS::SetGasPedalPercent(const std_msgs::Float64::ConstPtr
                                                &_msg)
{
  double min, max, percent, cmd;
  percent = clamp(static_cast<double>(_msg->data), 0.0, 1.0);
 /* DRCVehiclePlugin::GetGasPedalLimits(min, max);
  cmd = min + percent * (max - min);
  DRCVehiclePlugin::SetGasPedalState(cmd);*/
 ROS_INFO("gas information %f",percent);
}

////////////////////////////////////////////////////////////////////////////////
void vilma_MA_ROS::SetBrakePedalPercent(const std_msgs::Float64::ConstPtr
    &_msg)
{
  double min, max, percent, cmd;
  percent = clamp(static_cast<double>(_msg->data), 0.0, 1.0);
/*  DRCVehiclePlugin::GetBrakePedalLimits(min, max);
  cmd = min + percent * (max - min);
  DRCVehiclePlugin::SetBrakePedalState(cmd);*/
 ROS_INFO("brake_pedal information %f",percent);
}


////////////////////////////////////////////////////////////////////////////////
// Returns the ROS publish period (seconds).
double vilma_MA_ROS::GetRosPublishPeriod()
{
  return this->rosPublishPeriod;
}


////////////////////////////////////////////////////////////////////////////////
// Publish hand wheel, gas pedal, and brake pedal on ROS
void vilma_MA_ROS::RosPublishStates(const ros::WallTimerEvent& t)
{
//float a=t.current_real.sec +t.current_real.nsec/1000000000.0-():
 //ROS_INFO("Callback 1 triggered %d s %d ",t.current_real.sec,t.current_real.nsec);

double a=t.current_real.toSec()-lastcall;
   if(a>this->rosPublishPeriod/2)
	{
	lastcall=t.current_real.toSec();
	std::vector<double> u(10);
	u[0]=lastcall;
	u[1]=std::rand()/(1.0*RAND_MAX);
	clientMA.setU(u);
	std::string a=clientMA.run();
	if(a.size()){
		ROS_INFO("Request to MA %f ||| %f ",t.current_real.toSec(),t.profile.last_duration.toSec());
		ROS_ERROR(a.data());
		}
	else{
   std::vector<double> y=clientMA.getY();
    // Publish Float64 messages
    std_msgs::Float64 msg_steer, msg_brake, msg_gas, msg_hand_brake;
    msg_steer.data =y[0];
    this->pubHandWheelState.publish(msg_steer);
    msg_brake.data = y[1];
    this->pubBrakePedalState.publish(msg_brake);
    msg_gas.data = y[2];
    this->pubGasPedalState.publish(msg_gas);
    msg_hand_brake.data = y[3];
    this->pubHandBrakeState.publish(msg_hand_brake);
    // Publish Int8
    std_msgs::Int8  msg_direction;
    msg_direction.data = static_cast<int8_t>(y[4]);
    this->pubDirectionState.publish(msg_direction);
   }
}
else
	ROS_INFO("period very short %f ||| %f",a,t.profile.last_duration.toSec());
if(a>this->rosPublishPeriod*1.5)
	ROS_INFO("period very long %f ||| %f",a,t.profile.last_duration.toSec());


}

void vilma_MA_ROS::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosNode->ok())
  {
    this->queue.callAvailable(ros::WallDuration(timeout));
  }
}
