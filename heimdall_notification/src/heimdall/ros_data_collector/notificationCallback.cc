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
#include <heimdall_notification/ros_data_collector.h>

#include <ros/ros.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/exception/all.hpp>

#include <sstream>

namespace Heimdall
{
    void ROSDataCollector::notificationCallback(std_msgs::String::ConstPtr const &msg)
    {
        std::lock_guard<std::mutex> lock(d_mutex);

        if (d_new_notifications > 0)
            d_last_notification += "\n";
        
        using boost::property_tree::ptree;
        ptree tree;

        std::istringstream in(msg->data);

        std::string notification;
        try
        {
            read_json(in, tree);
            notification = tree.get<std::string>("message");
        }
        catch (boost::exception const &e)
        {
            std::string err = boost::diagnostic_information(e);
            ROS_ERROR("Invalid JSON received: %s", msg->data.c_str());
            ROS_DEBUG("Diagnostic information: %s", err.c_str());
            notification = msg->data;
        }

        d_last_notification += notification;
        ++d_new_notifications;
    }
}
