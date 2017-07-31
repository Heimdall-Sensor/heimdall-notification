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
#include <heimdall/email/address.h>

namespace Heimdall { namespace Email {
    bool Address::operator<(Address const &other) const
    {
        if (d_email == other.d_email)
        {
            if (
                d_type == other.d_type || 
                (d_type >= Type::TO && d_type <= Type::BCC && other.d_type >= Type::TO && other.d_type <= Type::BCC)
               )
            {
                return false;
            }
        }

        if (d_type != other.d_type)
            return d_type < other.d_type;

        return d_email < other.d_email;
    }
}}
