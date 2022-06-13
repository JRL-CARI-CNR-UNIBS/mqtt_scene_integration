/*
Copyright (c) 2020, Marco Faroni
CNR-STIIMA marco.faroni@stiima.cnr.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <ros/ros.h>
#include <ros/console.h>
#include <subscription_notifier/subscription_notifier.h>

#include <mqtt_scene_integration/OperatorState.h>
#include <mqtt_scene_integration/RobotState.h>

/* robot and human task request and response from task planner */
#include <task_planner_interface_msgs/MotionTaskExecutionRequestArray.h>
#include <task_planner_interface_msgs/MotionTaskExecutionFeedback.h>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_jfmx_communication");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ros::WallRate lp(20);

  ros_helper::SubscriptionNotifier<task_planner_interface_msgs::MotionTaskExecutionRequestArray> robot_req_notif(nh,"/robot_request_from_tp",1);
  ros_helper::SubscriptionNotifier<task_planner_interface_msgs::MotionTaskExecutionRequestArray> operator_req_notif(nh,"/operator_request_from_tpi",1);
  ros_helper::SubscriptionNotifier<task_planner_interface_msgs::MotionTaskExecutionFeedback> robot_res_notif(nh,"/robot_response",1);
  ros_helper::SubscriptionNotifier<mqtt_scene_integration::OperatorState> operator_res_notif(nh,"/JFMX/L1/sharework/station/operator",1);

  ros::Publisher robot_state_pub=nh.advertise<mqtt_scene_integration::RobotState>("/JFMX/L1/sharework/robot/status",1);
  ros::Publisher operator_state_pub=nh.advertise<mqtt_scene_integration::OperatorState>("/JFMX/L1/sharework/robot/operator",1);
  ros::Publisher operator_feedback_pub=nh.advertise<task_planner_interface_msgs::MotionTaskExecutionFeedback>("/operator_feedback_to_tpi",1);

  int operator_id = std::numeric_limits<int>::quiet_NaN();;
  std::pair<int,std::string> robot_id_name;
  robot_id_name.first = std::numeric_limits<int>::quiet_NaN();;
  robot_id_name.second = "";
  int number_of_slots=1;


  while (ros::ok())
  {
    ROS_INFO_THROTTLE(5,"ok");

    ros::spinOnce();

    if (operator_req_notif.isANewDataAvailable())
    {
      operator_id = operator_req_notif.getData().cmd_id;
      std::string task_name = operator_req_notif.getData().tasks.at(0).task_id;

      mqtt_scene_integration::OperatorState operator_state_msg;
      operator_state_msg.id = operator_id;

      std::string task_message;
      if (!nh.getParam(task_name+"/message",task_message))
      {
        ROS_ERROR("%s/message not defined",task_name.c_str());
        return 0;
      }

      int task_position=-1;
      if (!nh.getParam(task_name+"/position",task_position))
      {
        ROS_WARN("%s/position not defined",task_name.c_str());
      }

      int task_storage=-1;
      if (!nh.getParam(task_name+"/storage",task_storage))
      {
        ROS_WARN("%s/storage not defined",task_name.c_str());
      }

      std::vector<int> task_slots{-1};
      if (!nh.getParam(task_name+"/slot",task_slots))
      {
        ROS_WARN("%s/slot not defined",task_name.c_str());
      }

      operator_state_msg.message = task_message;
      operator_state_msg.position = task_position;
      operator_state_msg.storage = task_storage;
      operator_state_msg.slot = task_slots;

      operator_state_pub.publish(operator_state_msg);

    }

    if (operator_res_notif.isANewDataAvailable())
    {
      if (operator_res_notif.getData().id == operator_id)
      {
        task_planner_interface_msgs::MotionTaskExecutionFeedback operator_feedback_msg;
        operator_feedback_msg.cmd_id = operator_id;
        operator_feedback_msg.result = 1;

        operator_feedback_pub.publish(operator_feedback_msg);
        operator_id=std::numeric_limits<int>::quiet_NaN();
      }
    }

    if (robot_req_notif.isANewDataAvailable())
    {
      robot_id_name.first = robot_req_notif.getData().cmd_id;
      robot_id_name.second = robot_req_notif.getData().tasks.at(0).task_id;

      mqtt_scene_integration::RobotState robot_state_msg;

      std::string task_message;
      if (!nh.getParam(robot_id_name.second+"/message",task_message))
      {
        ROS_ERROR("%s/message not defined",robot_id_name.second.c_str());
        return 0;
      }

      int task_position=-1;
      if (!nh.getParam(robot_id_name.second+"/position",task_position))
      {
        ROS_WARN("%s/position not defined",robot_id_name.second.c_str());
      }

      int task_storage=-1;
      if (!nh.getParam(robot_id_name.second+"/storage",task_storage))
      {
        ROS_WARN("%s/storage not defined",robot_id_name.second.c_str());
      }

      number_of_slots=1;
      if (!nh.getParam(robot_id_name.second+"/number_of_slots",number_of_slots))
      {
        ROS_WARN("%s/number_of_slots not defined",robot_id_name.second.c_str());
      }



      robot_state_msg.message = task_message;
      robot_state_msg.position = task_position;
      robot_state_msg.storage = task_storage;
      robot_state_msg.slot.push_back(-1);

      robot_state_pub.publish(robot_state_msg);

    }

    if (robot_res_notif.isANewDataAvailable())
    {
      if (robot_res_notif.getData().cmd_id == robot_id_name.first)
      {
        mqtt_scene_integration::RobotState robot_state_msg;

        std::string task_message;
        if (!nh.getParam(robot_id_name.second+"/message",task_message))
        {
          ROS_ERROR("%s/message not defined",robot_id_name.second.c_str());
          return 0;
        }

        int task_position=-1;
        if (!nh.getParam(robot_id_name.second+"/position",task_position))
        {
          ROS_WARN("%s/position not defined",robot_id_name.second.c_str());
        }

        int task_storage=-1;
        if (!nh.getParam(robot_id_name.second+"/storage",task_storage))
        {
          ROS_WARN("%s/storage not defined",robot_id_name.second.c_str());
        }


        /* Look for slots chosen by the robot */

        std::vector<int> task_slots;
        try
        {
          std::vector<std::string> objects_manipulated;
          std::string tmp;
          if (!nh.getParam("/inbound_pick_server/manipulated_object_last/name",tmp))
          {
            ROS_WARN("/inbound_pick_server/manipulated_object_last/name not defined");
          }
          else
            objects_manipulated.push_back(tmp);

          if (!nh.getParam("/inbound_pick_server/manipulated_object_second_last/name",tmp))
          {
            ROS_WARN("/inbound_pick_server/manipulated_object_second_last/name not defined");
          }
          else
            objects_manipulated.push_back(tmp);

          for (unsigned int id_obj=0;id_obj<std::min(int (objects_manipulated.size()),number_of_slots);id_obj++)
          {
            int last_num;
            for (int idx=objects_manipulated.at(id_obj).length()-1;idx>=0;idx--)
            {
              if (objects_manipulated.at(id_obj)[idx]<='9' && objects_manipulated.at(id_obj)[idx]>='0')
                last_num = idx;
              else
                break;
            }
            task_slots.push_back(std::stoi(objects_manipulated.at(id_obj).substr(last_num,std::string::npos)));
          }

        }
        catch (const std::exception &ex)
        {
          ROS_ERROR("unable to decode slot number.");
          task_slots.push_back(-1);
        }

        if (task_message.compare("Loading")==0)
          task_message="EndLoad";
        else if (task_message.compare("Unloading")==0)
          task_message="EndUnload";

        robot_state_msg.message = task_message;
        robot_state_msg.position = task_position;
        robot_state_msg.storage = task_storage;
        robot_state_msg.slot = task_slots;

        robot_state_pub.publish(robot_state_msg);

      }
    }

    lp.sleep();
  }


  return 0;


}
