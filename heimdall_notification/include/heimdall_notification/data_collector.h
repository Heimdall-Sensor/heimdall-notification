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
#ifndef __INCLUDED_HEIMDALL_DATA_COLLECTOR_H_
#define __INCLUDED_HEIMDALL_DATA_COLLECTOR_H_

#include <memory>
#include <string>
#include "parameters.h"

namespace Heimdall
{
    class DataCollector
    {
        public:
            DataCollector(std::shared_ptr<Parameters> parameters);
            virtual ~DataCollector();
            virtual bool shouldNotify(std::string &reason, size_t msec) = 0;
            void setParameters(std::shared_ptr<Parameters> parameters);

        protected:
            std::shared_ptr<Parameters> d_params;
    };

    inline DataCollector::DataCollector(std::shared_ptr<Parameters> parameters)
    {
        setParameters(parameters);
    }

    inline DataCollector::~DataCollector()
    {}

    inline void DataCollector::setParameters(std::shared_ptr<Parameters> parameters)
    {
        d_params = parameters;
    }
}

#endif
