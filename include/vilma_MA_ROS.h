/**
 *
	 @brief This is the library with all the topics generated (published) and that receive (suscribers) the microautobox
based on the code of the plugin DRCVehicleROSPlugin of the drcsim 2.7 of Open Source Robotics Foundation

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


#ifndef VILMA_MA_ROS_HH
#define VILMA_MA_ROS_HH
#include <boost/thread.hpp>
//#include <boost/thread/mutex.hpp>
#include "maudp.h"
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/advertise_options.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
/**
This is the class with all the topics generated (published) and that receive (suscribers) the microautobox
it use @ref maudp to comunicate with eh microautobox.
*/
class vilma_MA_ROS
{
    /// \brief Constructor.
    public: vilma_MA_ROS();

    /// \brief Destructor.
    public: virtual ~vilma_MA_ROS();
    /// Returns the ROS publish period (seconds).
    public: double GetRosPublishPeriod();
/// \brief Publish the steering and pedal states on ROS topics.
    private: void RosPublishStates(const ros::WallTimerEvent&);



    /// \brief Sets the state of the direction switch.
    /// \param[in] _msg Desired direction state as Int8 message.
    ///            Use -1 for REVERSE, 0 for NEUTRAL, 1 for FORWARD.
    public: void SetDirectionState(const std_msgs::Int8::ConstPtr &_msg);

    /// \brief Set the steering wheel angle; this will also update the front
    ///        wheel steering angle.
    /// \param[in] _msg ROS std_msgs::Float64 message.
    public: void SetHandWheelState(const std_msgs::Float64::ConstPtr &_msg);

    /// \brief Specify the desired hand brake position, as a percentage of
    ///        the range of travel.
    /// \param[in] _msg ROS std_msgs::Float64 message, with data representing
    ///                 the desired percent.
    public: void SetHandBrakePercent(const std_msgs::Float64::ConstPtr &_msg);

    /// \brief Specify the desired gas pedal position, as a percentage of
    ///        the range of travel.
    /// \param[in] _msg ROS std_msgs::Float64 message, with data representing
    ///                 the desired percent.
    public: void SetGasPedalPercent(const std_msgs::Float64::ConstPtr &_msg);

    /// \brief Specify the desired brake pedal position, as a percentage of
    ///        the range of travel.
    /// \param[in] _msg ROS std_msgs::Float64 message, with data representing
    ///                 the desired percent.
    public: void SetBrakePedalPercent(const std_msgs::Float64::ConstPtr &_msg);


    // ros stuff
    private: ros::NodeHandle* rosNode;
    private: ros::CallbackQueue queue;
    private: void QueueThread();
    private: boost::thread callbackQueueThread;
    private: ros::Publisher pubBrakePedalState;
    private: ros::Publisher pubGasPedalState;
    private: ros::Publisher pubHandWheelState;
    private: ros::Publisher pubHandBrakeState;
    private: ros::Publisher pubDirectionState;
    private: ros::Subscriber subBrakePedalCmd;
    private: ros::Subscriber subGasPedalCmd;
    private: ros::Subscriber subHandWheelCmd;
    private: ros::Subscriber subHandBrakeCmd;
    private: ros::Subscriber subDirectionCmd;
    private: double rosPublishPeriod;
   // private: ros::Time lastRosPublishTime;
    private:  microautobox::maudp  clientMA;
    private:   ros::WallTimer timer;
    private:   double lastcall;
private: inline double clamp(double x, double a, double b)
{

    return x < a ? a : (x > b ? b : x);

}



};
#endif
