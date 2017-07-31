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
#include <iostream>
#include <vector>

#include <heimdall_notification/ros_parameters.h>
#include <heimdall_notification/sms_notifier.h>
#include <heimdall_notification/email_notifier.h>
#include <heimdall_notification/ros_data_collector.h>

#include <ros/ros.h>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "notification");
    ros::NodeHandle nh("heimdall");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    try
    {
        shared_ptr<Heimdall::Parameters> parameters(new Heimdall::ROSParameters(nh, "/heimdall/notification/"));
        unique_ptr<Heimdall::DataCollector> collector(new Heimdall::ROSDataCollector(nh, parameters));

        // Set up notifiers according to configuration
        vector<shared_ptr<Heimdall::Notifier>> notifiers;
        vector<string> notifier_names = parameters->getArray("notifiers");
        for (auto ptr = notifier_names.begin(); ptr != notifier_names.end(); ++ptr)
        {
            shared_ptr<Heimdall::Parameters> notifier_params(new Heimdall::ROSParameters(nh, "/heimdall/notification/" + *ptr));
            string notifier_type = notifier_params->get("type");
            
            if (notifier_type == "sms")
            {
                notifiers.push_back(shared_ptr<Heimdall::Notifier>(new Heimdall::SMSNotifier(notifier_params)));
                ROS_ERROR("Set up SMS notifier");
            }
            else if (notifier_type == "email")
            {
                notifiers.push_back(shared_ptr<Heimdall::Notifier>(new Heimdall::EmailNotifier(notifier_params)));
                ROS_ERROR("Set up email notifier");
            }
            else
                ROS_ERROR("Unknown notifier type: %s", notifier_type.c_str());
        }

        if (notifiers.empty())
        {
            ROS_ERROR("No notifiers configured. No need to run");
            spinner.stop();
            ros::shutdown();
            return 1;
        }

        ROS_ERROR("Set up %d notifiers", notifiers.size());

        while (ros::ok())
        {
            string msg;
            if (collector->shouldNotify(msg, 1000))
            {
                ROS_ERROR("Delivering notification: %s", msg.c_str());

                for (auto ptr = notifiers.begin(); ptr != notifiers.end(); ++ptr)
                    (*ptr)->sendNotification(msg);
            }
        }
    }
    catch (exception &e)
    {
        ROS_FATAL("Fatal error: %s", e.what());
    }
}
