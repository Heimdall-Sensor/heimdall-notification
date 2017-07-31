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
#ifndef __INCLUDED_HEIMDALL_EMAIL_H_
#define __INCLUDED_HEIMDALL_EMAIL_H_

#include <string>
#include <set>
#include <ctime>

namespace Heimdall { namespace Email {
    class Message
    {
        public:
            Message();
            ~Message();

            void addAddress(Address &&address);
            void setBody(std::string const &body);
            void setBody(mimetic::MimeEntity const &body);
            void setTime(tm *sendDate);
            void setSubject(std::string const &subject);
            void setSubject(std::string &&subject);
            bool isSent();
            bool send();

        protected:
            std::set<Address> d_addresses;
            std::string d_subject;
            std::string d_body;
            tm *d_send_date;

            bool d_sent;
    };

    inline Message::Message()
    :
        d_send_date(nullptr),
        d_sent(false)
    {}

    inline Message::~Message()
    {
        if (d_send_date != nullptr)
            delete d_send_date;
    }

    inline bool Message::isSent()
    {
        return d_sent;
    }
}}
#endif
