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

#include "x_vio_ros/communication.h"
#include <chrono>

using namespace x;

Communication::~Communication() {
    run_thr_ = false;
    thr_.join();
}

Communication::Communication() : uav_name_(ros::this_node::getNamespace()) {
    node_name_[0] = ros::this_node::getName();

    if (!ros::master::execute("getUri", node_name_, result_, payload_, true)) {
        throw std::runtime_error("Communication to ROS MASTER failed. Make sure the ROS MASTER is running...");
    }
    master_uri_ = std::string(payload_);
    std::cout << "Comunication to ROS MASTER successed.\nROS_MASTER_URI: "
              << master_uri_ << std::endl;
}

void Communication::start(Node *node, ros::NodeHandle &nh) {
#ifndef REQUEST_COMM
    auto pub = nh.advertise<x_vio_ros::MessageUAV>(uav_name_ + "/message", 2);
    pubs_multi_uav_.emplace_back(pub);
#endif
    thr_ = std::thread(std::bind(&x::Communication::getTopics, this, node, nh));
}

void Communication::getOtherUAV(std::set<std::string> &other_messages_topics,
                                std::set<std::string> &other_request_topics) {
    std::string subgraph;
    ros::master::V_TopicInfo topics;

    XmlRpc::XmlRpcValue args;
    args[0] = ros::this_node::getName();
    args[1] = subgraph;

    if (!ros::master::execute("getPublishedTopics", args, result_, payload_,
                              true)) {
        throw std::runtime_error("Cannot perform \"getPublishedTopics\". Communication to ROS MASTER failed.");
    }
    other_messages_topics.clear();
    other_request_topics.clear();
    for (int i = 0; i < payload_.size(); ++i) {
#ifdef REQUEST_COMM
        if (std::string(payload_[i][0]).find(uav_name_) != std::string::npos &&
            std::string(payload_[i][0])
                    .compare(uav_name_.size(), uav_name_.size(),
                             uav_name_)  // TODO make this work with more than 10 UAVs
            // -> split string
            &&
            std::string(payload_[i][1]).find("MessageUAV") != std::string::npos) {
          other_messages_topics.insert(std::string(payload_[i][0]));
        }
        if (std::string(payload_[i][0]).find(uav_name_) == std::string::npos &&
            std::string(payload_[i][1]).find("RequestUAV") != std::string::npos) {
          other_request_topics.insert(std::string(payload_[i][0]));
        }
#else
        if (std::string(payload_[i][0]).find(uav_name_) == std::string::npos &&
            std::string(payload_[i][1]).find("MessageUAV") != std::string::npos) {
            other_messages_topics.insert(std::string(payload_[i][0]));
        }
#endif
    }
}

void Communication::getTopics(Node *node, ros::NodeHandle &nh) {
    while (run_thr_) {
        std::set<std::string> other_messages_topics_now, other_request_topics_now;
        getOtherUAV(other_messages_topics_now, other_request_topics_now);
        std::set<std::string> diff;

        std::set_difference(
                other_messages_topics_now.begin(), other_messages_topics_now.end(),
                other_messages_topics_.begin(), other_messages_topics_.end(),
                std::inserter(diff, diff.begin()));
        ros::Subscriber com_sub;

        for (const auto &topic: diff) {
            com_sub = nh.subscribe(topic, 10, &Node::multiUavMessageCallback, node);
            sub_message_list_.emplace_back(com_sub);
            other_messages_topics_.insert(topic);
            std::cout << "Subscribed to : " << topic << std::endl;
        }
#ifdef REQUEST_COMM
        diff.clear();
        std::set_difference(
            other_request_topics_now.begin(), other_request_topics_now.end(),
            other_request_topics_.begin(), other_request_topics_.end(),
            std::inserter(diff, diff.begin()));
        for (auto topic : diff) {
          com_sub = nh.subscribe(topic, 10, &Node::multiUavRequestCallback, node);
          sub_request_list_.emplace_back(com_sub);
          other_request_topics_.insert(topic);
          std::cout << "Subscribed to : " << topic << std::endl;

          auto pub = nh.advertise<x_vio_ros::MessageUAV>(
              topic.substr(0, 5) + uav_name_ + "_message", 2);
          pubs_multi_uav_.emplace_back(pub);
          pubs_multi_uav_idxs_.push_back(std::stoi(topic.substr(
              4, 5)));  // TODO change this to work with more than 10 UAVs
          std::cout << "Created publisher: "
                    << (topic.substr(0, 5) + uav_name_ + "_message") << std::endl;
        }
#endif
        std::this_thread::sleep_for(std::chrono::seconds(10)); // sleep for 10 secs
    }
}

void Communication::publish(int uav_id, x_vio_ros::MessageUAV &msg) {
    std::lock_guard<std::mutex> lck(mtx_);
#ifndef REQUEST_COMM
    pubs_multi_uav_[0].publish(msg);

#else
    for (size_t i = 0; i < pubs_multi_uav_idxs_.size(); i++) {
      if (pubs_multi_uav_idxs_[i] == uav_id) {
        pubs_multi_uav_[i].publish(msg);
        break;
      }
    }
#endif
}
