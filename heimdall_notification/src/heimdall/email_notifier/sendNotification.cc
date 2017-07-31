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
#include <heimdall_notification/email_notifier.h>

#include <sstream>
#include <curl/curl.h>
#include <cstring>
#include <ctime>

#include <iostream>

namespace Heimdall
{
    struct SendingData
    {
        char const *msg;
        size_t position;
        size_t length;
    };

    static size_t mail_helper(void *ptr, size_t size, size_t nmemb, void *user_data)
    {
        SendingData *data = reinterpret_cast<SendingData *>(user_data);

        if (data->position >= data->length)
            return 0;

        size_t pos = data->position;
        size_t bytes = size * nmemb;

        if (bytes > data->length - pos)
            bytes = data->length - pos;

        memcpy(ptr, data->msg + data->position, bytes);
        data->position += bytes;
        return bytes;
    }

    void EmailNotifier::sendNotification(std::string const &msg)
    {
        if (d_throttler.throttle(msg))
        {
            std::cerr << "Throttled notification '" << msg << "'\n";
            return;
        }

        // Parameters
        std::string smtp_host = d_params->get("smtp_host", "localhost");
        std::string smtp_port = d_params->get("smtp_port", "25");
        std::string smtp_tls = d_params->get("smtp_tls", "0");
        std::string mail_from = d_params->get("mail_from");
        std::string mail_to = d_params->get("mail_recipient");
        std::string mail_subject = d_params->get("mail_subject", "Heimdall notification");

        std::string smtp_user = d_params->get("smtp_user");
        std::string smtp_password = d_params->get("smtp_password");
        std::string verbose_str = d_params->get("verbose");

        bool verbose = verbose_str == "1" || verbose_str == "true";

        // Compile the URL
        std::string smtp_url("smtp://");
        smtp_url += smtp_host + ":" + smtp_port + "/";

        CURL *ch = curl_easy_init();
        curl_easy_setopt(ch, CURLOPT_URL, smtp_url.c_str());

        if (smtp_user.length() > 0 && smtp_password.length() > 0)
        {
            curl_easy_setopt(ch, CURLOPT_USERNAME, smtp_user.c_str());
            curl_easy_setopt(ch, CURLOPT_PASSWORD, smtp_password.c_str());
            if (verbose)
                std::cerr << "Using authentication to SMTP server with username " << smtp_user << "\n";
        }

        // Enable SSL if requested
        if (smtp_tls == "1" || smtp_tls == "true")
        {
            curl_easy_setopt(ch, CURLOPT_USE_SSL, (long)CURLUSESSL_ALL);
            if (verbose)
                std::cerr << "Enabling TLS\n";
        }

        curl_easy_setopt(ch, CURLOPT_MAIL_FROM, mail_from.c_str());

        struct curl_slist *recipients = nullptr;
        recipients = curl_slist_append(recipients, mail_to.c_str());
        curl_easy_setopt(ch, CURLOPT_MAIL_RCPT, recipients);

        // Date
        time_t raw_time;
        time(&raw_time);

        tm *timeinfo;
        timeinfo = localtime(&raw_time);

        char date[256];
        strftime(date, 256, "%a, %d %b %Y %T %z", timeinfo);

        // Compile the message
        std::ostringstream email_msg;
        email_msg << "Date: " << date << "\r\n";
        email_msg << "To: " << mail_to << "\r\n";
        email_msg << "From: " << mail_from << "\r\n";
        email_msg << "Subject: " << mail_subject << "\r\n";
        email_msg << "\r\n";
        email_msg << msg << "\r\n";

        // Attach the payload to the message
        std::string email_str = email_msg.str();
        char const *email_cstr = email_str.c_str();

        //std::cerr << "Full message: \n" << email_cstr << std::endl;

        SendingData data;
        data.position = 0;
        data.msg = email_cstr;
        data.length = email_str.length();

        curl_easy_setopt(ch, CURLOPT_READFUNCTION, mail_helper);
        curl_easy_setopt(ch, CURLOPT_READDATA, &data);
        curl_easy_setopt(ch, CURLOPT_INFILESIZE, static_cast<long>(data.length));
        curl_easy_setopt(ch, CURLOPT_UPLOAD, 1L);

        if (verbose)
            curl_easy_setopt(ch, CURLOPT_VERBOSE, 1L);

        // Execute the request and free the result
        CURLcode res = CURLE_OK;
        
        res = curl_easy_perform(ch);

         if (res != CURLE_OK)
             std::cerr << "Failed to send e-mail notification: " << curl_easy_strerror(res) << std::endl;
         else
            std::cerr << "Notification '" << msg << "' has been sent using e-mail from " << mail_from << " to " << mail_to << std::endl;

        curl_slist_free_all(recipients);
        curl_easy_cleanup(ch);
    }
}
