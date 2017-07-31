/*
    Heimdall Notification handles and dispatches notifications by various
    media, including SMS and e-mail.
    Copyright (C) 2017 Christof Oost, Amir Shantia, Ron Snijders, Egbert van der Wal

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef __INCLUDED_HEIMDALL_ROS_DATA_COLLECTOR_H_
#define __INCLUDED_HEIMDALL_ROS_DATA_COLLECTOR_H_

#include <string>
#include "data_collector.h"

#include <std_msgs/String.h>
#include <mutex>

#include <ros/ros.h>

namespace Heimdall
{
    class ROSDataCollector: public DataCollector
    {
        public:
            ROSDataCollector(ros::NodeHandle &nh, std::shared_ptr<Parameters> parameters);
            virtual bool shouldNotify(std::string &reason, size_t msec);

            void notificationCallback(std_msgs::String::ConstPtr const &msg);

        protected:
            ros::NodeHandle &d_nh;
            ros::Subscriber d_notification_sub;
            std::string d_last_notification;
            size_t d_new_notifications;
            std::mutex d_mutex;
    };
}

#endif
