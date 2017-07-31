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
    std::string ROSParameters::get(std::string const &key)
    {
        return this->get(key, d_empty);
    }

    std::vector<std::string> ROSParameters::getArray(std::string const &key)
    {
        std::vector<std::string> result;
        std::string param_name = d_namespace + key;
        ros::param::get(param_name, result);
        return result;
    }

    std::string ROSParameters::get(std::string const &key, std::string const &def)
    {
        std::string sVal;
        int iVal;
        bool bVal;

        std::string param_name = d_namespace + key;

        if (ros::param::get(param_name, sVal))
            return sVal;

        if (ros::param::get(param_name, iVal))
            return std::to_string(iVal);

        if (ros::param::get(param_name, bVal))
            return std::to_string(bVal);

        return def;
    }
}
