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
#ifndef __INCLUDED_HEIMDALL_EMAIL_ADDRESS_H_
#define __INCLUDED_HEIMDALL_EMAIL_ADDRESS_H_

#include <string>
#include <ctime>

namespace Heimdall { namespace Email {
    class Address
    {
        public:
            enum Type 
            {
                FROM,
                REPLY_TO,
                TO,
                CC,
                BCC
            };

            Address(std::string const &name, std::string const &email, Type type);
            std::string const &getAddress();
            std::string const &getName();
            Header getHeader() const;
            bool operator<(Address const &other) const;

        private:
            std::string d_name;
            std::string d_email;
            Type d_type;
    };

    inline std::string const &Address::getEmail()
    {
        return d_email;
    }

    inline std::string const &Address::getName()
    {
        return d_name;
    }
}}
