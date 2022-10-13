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

#include <dynamic_reconfigure/server.h>
#include <memory>
#include <ros/ros.h>
#include <x_vio_ros/communication.h>
#include <x_vio_ros/node.h>
#include <thread>

/** Run (multithreaded) XIO within a ROS node
 */
int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "x_vio");

    // Start node
    ros::NodeHandle nh;

    // Set up xVIO within this node

#ifdef MULTI_UAV
    std::shared_ptr<x::Communication> communication =
            std::make_shared<x::Communication>();
    x::Node vio_node(nh, communication);
#else
    x::Node vio_node(nh);
#endif

    std::vector<ros::Subscriber>
            other_topics_list_; // list of the other UAV topics we are going to
    // subscribe to
#ifdef MULTI_UAV
    communication->start(&vio_node, nh);
#ifdef MULTI_THREAD
    std::thread other_thread =
            std::thread([ObjectPtr = &vio_node] { ObjectPtr->processOther(); });
    std::thread other_request_thread =
            std::thread([ObjectPtr = &vio_node] { ObjectPtr->processOtherRequest(); });
#endif
#endif

// Set up visual thread
#ifdef MULTI_THREAD
    std::thread visual_thread =
            std::thread(&x::Node::processVision, &vio_node);
    ROS_INFO("Thread started");
#endif

    // Set up dynamic reconfigure
    dynamic_reconfigure::Server<x_vio::xvioConfig> srv;
    dynamic_reconfigure::Server<x_vio::xvioConfig>::CallbackType f;
    f = boost::bind(&x::Node::reconfigure, &vio_node, _1, _2);
    srv.setCallback(f);

    // Cycle callbacks
    ros::spin();

#ifdef MULTI_THREAD
    vio_node.stopNode();

    // Terminate visual thread
    // Interrupt callback thread, otherwise node closing will hang, since join
    // waits for the thread to finish its work.
    visual_thread.join();
#ifdef MULTI_UAV
    other_thread.join();
    other_request_thread.join();
#endif
    ROS_INFO("Thread stopped");
#endif

    return 0;
}
