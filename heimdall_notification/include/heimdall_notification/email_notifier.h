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
#ifndef __INCLUDED_HEIMDALL_EMAIL_NOTIFIER_H_
#define __INCLUDED_HEIMDALL_EMAIL_NOTIFIER_H_

#include "notifier.h"
#include "throttler.h"

#include <string>

namespace Heimdall
{
    class EmailNotifier: public Notifier
    {
        public:
            EmailNotifier(std::shared_ptr<Parameters> parameters);
            ~EmailNotifier();
            void sendNotification(std::string const &message);

        private:
            Throttler d_throttler;
    };
}
#endif
