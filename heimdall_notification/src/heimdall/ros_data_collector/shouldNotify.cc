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

namespace Heimdall
{
    bool ROSDataCollector::shouldNotify(std::string &msg, size_t msec)
    {
        ros::Duration timeout(0, msec * 1000000);
        ros::Time target = ros::Time::now() + timeout;

        while (ros::Time::now() < target)
        {
            {
                std::lock_guard<std::mutex> lock(d_mutex);
                if (d_new_notifications > 0)
                {
                    msg = d_last_notification;
                    d_new_notifications = 0;
                    d_last_notification = "";
                    return true;
                }
            }
            ros::Duration(0.1).sleep();
        }

        return false;
    }
}
