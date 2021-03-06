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
#ifndef __INCLUDED_HEIMDALL_ROS_PARAMETERS_H_
#define __INCLUDED_HEIMDALL_ROS_PARAMETERS_H_

#include <string>

#include "parameters.h"

namespace ros 
{
    class NodeHandle;
}

namespace Heimdall
{
    class ROSParameters: public Parameters
    {
        public:
            ROSParameters(ros::NodeHandle &nh);
            ROSParameters(ros::NodeHandle &nh, std::string const &ns);
            virtual std::string operator[](std::string const &key);
            virtual std::string get(std::string const &key);
            virtual std::string get(std::string const &key, std::string const &def);
            virtual std::vector<std::string> getArray(std::string const &key);

        protected:
            std::string d_empty;
            std::string d_namespace;
            ros::NodeHandle &d_nh;
    };
}

#endif
