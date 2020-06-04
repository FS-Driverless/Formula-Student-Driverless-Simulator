#include <iostream>
#include <fstream>
#include <jsoncpp/json/json.h>
#include <curl/curl.h>

int main()
{
    std::ifstream file("../../../../config/team_config.json");
    Json::Reader reader;
    Json::Value obj;
    reader.parse(file, obj);
    std::string access_token = obj["access_token"].asString();

    CURL *curl;
    CURLcode res;

    curl_global_init(CURL_GLOBAL_ALL);
    curl = curl_easy_init();

    std::string json_obj = "{\"access_token\" : \"" + access_token + "\" , \"sender\" : \"AS\" }";

    struct curl_slist *headers = NULL;
    headers = curl_slist_append(headers, "Accept: application/json");
    headers = curl_slist_append(headers, "Content-Type: application/json");
    headers = curl_slist_append(headers, "charsets: utf-8");

    curl_easy_setopt(curl, CURLOPT_URL, "http://localhost:5000/mission/stop");
    curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "POST");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_obj.c_str());

    res = curl_easy_perform(curl);
    std::cout << res << std::endl;

    curl_easy_cleanup(curl);
    curl_global_cleanup();

    return 0;
}
