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
#include <heimdall_notification/ros_parameters.h>

#include <ros/ros.h>

namespace Heimdall
{
    ROSParameters::ROSParameters(ros::NodeHandle &nh)
    :
        d_nh(nh)
    {}

    ROSParameters::ROSParameters(ros::NodeHandle &nh, std::string const &ns)
    :
        d_nh(nh)
    {
        if (ns.size() > 0 && (*ns.rbegin()) != '/')
            d_namespace = ns + '/';
        else
            d_namespace = ns;
    }
}

