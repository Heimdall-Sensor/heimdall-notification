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
#ifndef __INCLUDED_HEIMDALL_NOTIFICATION_THROTTLER_H_
#define __INCLUDED_HEIMDALL_NOTIFICATION_THROTTLER_H_
#include <list>
#include <sys/time.h>

namespace Heimdall { 
    class Throttler
    {
        private:
            struct Notification
            {
                std::string notification;
                unsigned long receive_time;
            };

        public: 
            Throttler(std::shared_ptr<Parameters> parameters);        
            bool throttle(std::string const &msg);

        private:
            std::list<Notification> d_recent_notifications;
            size_t d_equal_message_timeout;
            size_t d_message_timeout;
    };

    inline Throttler::Throttler(std::shared_ptr<Parameters> parameters)
    :
        d_equal_message_timeout(std::stoi(parameters->get("repeat_throttle"))),
        d_message_timeout(std::stoi(parameters->get("message_throttle")))
    {}

    inline bool Throttler::throttle(std::string const &msg)
    {
        Notification n;
        n.notification = msg;
        n.receive_time = time(nullptr);

        // Throttle messages to once per 60 seconds
        unsigned long int equal_threshold = n.receive_time - d_equal_message_timeout;
        unsigned long int msg_threshold = n.receive_time - d_message_timeout;

        unsigned long int remove_threshold = std::min(equal_threshold, msg_threshold);

        // Remove old messages
        while (not d_recent_notifications.empty() && d_recent_notifications.front().receive_time < remove_threshold)
            d_recent_notifications.pop_front();

        // Check for duplicates
        for (auto ptr = d_recent_notifications.rbegin(); ptr != d_recent_notifications.rend(); ++ptr)
        {
            if (ptr->receive_time > msg_threshold)
                return true;
            
            if (ptr->notification == n.notification && ptr->receive_time > equal_threshold)
                return true;
        }

        d_recent_notifications.push_back(n);
        return false;
    }
}
#endif

