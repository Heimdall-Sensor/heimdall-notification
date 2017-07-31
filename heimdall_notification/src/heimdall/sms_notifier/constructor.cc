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
#include <heimdall_notification/sms_notifier.h>

#include <curl/curl.h>
#include <stdexcept>

using std::runtime_error;

namespace Heimdall
{
    SMSNotifier::SMSNotifier(std::shared_ptr<Parameters> parameters)
    :
        Notifier(parameters),
        d_throttler(parameters)
    {
        curl_global_init(CURL_GLOBAL_SSL);

        // Validate parameters
        std::string tmp = parameters->get("cm_token");
        if (tmp.length() == 0)
            throw runtime_error("Please a CM Token for SMS notifications");

        tmp = parameters->get("sms_sender");
        if (tmp.length() == 0)
            throw runtime_error("Please provide a SMS sender name");

        tmp = parameters->get("sms_recipient");
        if (tmp.length() == 0)
            throw runtime_error("Please provide a SMS recipient");
    }
}
