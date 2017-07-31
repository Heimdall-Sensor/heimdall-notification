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

namespace Heimdall
{
    ROSDataCollector::ROSDataCollector(ros::NodeHandle &nh, std::shared_ptr<Parameters> parameters)
    :
        DataCollector(parameters),
        d_nh(nh),
        d_new_notifications(0)
    {
        std::string topic = parameters->get("notification_topic", "/heimdall/notification");
        d_notification_sub = d_nh.subscribe(topic, 1,  &ROSDataCollector::notificationCallback, this);
    }
}
