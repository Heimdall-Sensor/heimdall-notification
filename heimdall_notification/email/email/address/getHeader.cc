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

namespace Heimdall
{
    namespace Email
    {
        Header Address::getHeader() const
        {
            std::string key;
            switch (d_type)
            {
                case Type::TO:
                    key = "To";
                    break;
                case Type::CC:
                    key = "Cc";
                    break;
                case Type::BCC:
                    key = "Bcc";
                    break;
                case Type::FROM:
                    key = "From";
                    break;
                case Type::REPLY_TO:
                    key = "Reply-to";
                    break;
            }
            std::string val;
            if (!d_name.empty())
            {
                val = d_name + " <";
                val += d_email;
                val += ">";
            }
            else
                val = d_email;

            return Header(key, val);
        }
    }
}
