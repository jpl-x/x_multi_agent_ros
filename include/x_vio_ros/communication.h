/*
 * Copyright 2020 California  Institute  of Technology (“Caltech”)
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
 */
#ifdef MULTI_UAV
#ifndef COMMUNICATION_X_
#define COMMUNICATION_X_

#include <ros/master.h>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <string>
#include <vector>

#include "x_vio_ros/RequestUAV.h"
#include "x_vio_ros/MessageUAV.h"
#include "x_vio_ros/node.h"

namespace x {
    class Node;

    class Communication {
    public:
        Communication();

        ~Communication();

        void start(Node *node, ros::NodeHandle &nh);

        void getOtherUAV(std::set<std::string> &other_messages_topics,
                         std::set<std::string> &other_request_topics);

        void getTopics(Node *node, ros::NodeHandle &nh);

        void publish(int uav_id, x_vio_ros::MessageUAV &msg);

    private:
        std::vector<ros::Subscriber> sub_message_list_, sub_request_list_;
        std::set<std::string> other_messages_topics_, other_request_topics_;
        XmlRpc::XmlRpcValue node_name_, result_, payload_;
        std::string master_uri_, uav_name_;

        std::mutex mtx_;
        std::thread thr_;
        std::atomic_bool run_thr_ = true;

        std::vector<ros::Publisher> pubs_multi_uav_;
        std::vector<int> pubs_multi_uav_idxs_;
        std::vector<x_vio_ros::MessageUAV> stored_msgs;
    };
}  // namespace x

#endif
#endif