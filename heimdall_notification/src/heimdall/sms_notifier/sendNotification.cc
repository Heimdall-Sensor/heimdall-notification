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

#include <sstream>
#include <curl/curl.h>

#include <iostream>

namespace Heimdall
{
    void SMSNotifier::sendNotification(std::string const &msg)
    {
        if (d_throttler.throttle(msg))
            return;

        CURL *ch = curl_easy_init();
        curl_easy_setopt(ch, CURLOPT_URL, "https://sgw01.cm.nl/gateway.ashx");

        // Enable this to get verbose output from CURL
        //curl_easy_setopt(ch, CURLOPT_VERBOSE, 1);

        // Parameters
        std::string cm_token = d_params->get("cm_token");
        std::string cm_reference = d_params->get("cm_reference");

        std::string sender = d_params->get("sms_sender");
        std::string recipient = d_params->get("sms_recipient");
        std::string verbose_str = d_params->get("verbose");

        bool verbose = verbose_str == "1" || verbose_str == "true";

        // Compile the message
        std::ostringstream xml_msg;
        xml_msg << "<?xml version=\"1.0\"?>\n";
        xml_msg << "<MESSAGES><AUTHENTICATION><PRODUCTTOKEN>";
        xml_msg << cm_token;
        xml_msg << "</PRODUCTTOKEN></AUTHENTICATION>";
        xml_msg << "<MSG><FROM>" << sender << "</FROM>";
        xml_msg << "<TO>" << recipient << "</TO>";
        xml_msg << "<BODY>" << msg << "</BODY>";
        xml_msg << "<REFERENCE>" << cm_reference << "</REFERENCE>";
        xml_msg << "</MSG></MESSAGES>\n";

        // Attach the payload to the message
        std::string xml_str = xml_msg.str();
        char const *xml_cstr = xml_str.c_str();
        std::cout << "Full message: " << xml_cstr << std::endl;

        curl_easy_setopt(ch, CURLOPT_POSTFIELDS, xml_cstr);
        curl_easy_setopt(ch, CURLOPT_POSTFIELDSIZE, xml_str.length());

        // Set the correct content-type
        struct curl_slist *headers = nullptr;
        headers = curl_slist_append(headers, "Content-Type: application/xml");
        curl_easy_setopt(ch, CURLOPT_HTTPHEADER, headers);

        if (verbose)
            curl_easy_setopt(ch, CURLOPT_VERBOSE, 1L);

        // Execute the request and free the result
        CURLcode res = curl_easy_perform(ch);
        if (res != CURLE_OK)
             std::cerr << "Failed to send SMS notification using curl: " << curl_easy_strerror(res) << std::endl;
        else
            std::cout << "Notification has been sent using SMS from: " << sender << " to " << recipient << std::endl;

        curl_slist_free_all(headers);
        curl_easy_cleanup(ch);
    }
}
