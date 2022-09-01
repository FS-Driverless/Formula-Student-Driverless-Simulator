// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_settings_hpp
#define msr_airlib_settings_hpp

#include "common_utils/Utils.hpp"

STRICT_MODE_OFF
// this json library is not strict clean
//TODO: HACK!! below are added temporariliy because something is defining min, max macros
//#undef max
#undef min
#include "common_utils/json.hpp"
STRICT_MODE_ON

#include <string>
#include <mutex>
#include "common_utils/FileSystem.hpp"

namespace msr { namespace airlib {

class Settings {
private:
    nlohmann::json doc_;

private:
    static std::mutex& getFileAccessMutex()
    {
        static std::mutex file_access;
        return file_access;
    }

public:
    static Settings& singleton() {
        static Settings instance;
        return instance;
    }

    static Settings& loadJSonString(const std::string& json_str)
    {
        if (json_str.length() > 0) {
            std::stringstream ss;
            ss << json_str;
            ss >> singleton().doc_;
        } else {
            throw std::invalid_argument("Invalid settings.json: json string empty.");
        }
        return singleton();
    }

    bool getChild(const std::string& name, Settings& child) const
    {
        if (doc_.count(name) == 1 && 
            ( doc_[name].type() == nlohmann::detail::value_t::object ||
                doc_[name].type() == nlohmann::detail::value_t::array
            )) {
            child.doc_ = doc_[name].get<nlohmann::json>();
            return true;
        }
        return false;
    }

    size_t size() const {
        return doc_.size();
    }

    template<typename Container>
    void getChildNames(Container& c) const
    {
        for (auto it = doc_.begin(); it != doc_.end(); ++it) {
            c.push_back(it.key());
        }
    }

    bool getChild(size_t index, Settings& child) const
    {
        if (doc_.size() > index && 
            ( doc_[index].type() == nlohmann::detail::value_t::object ||
                doc_[index].type() == nlohmann::detail::value_t::array
            )) {

            child.doc_ = doc_[index].get<nlohmann::json>();
            return true;
        }
        return false;
    }

    std::string getString(const std::string& name, std::string defaultValue) const
    {
        if (doc_.count(name) == 1) {
            return doc_[name].get<std::string>();
        }
        else {
            return defaultValue;
        }
    }

    double getDouble(const std::string& name, double defaultValue) const
    {
        if (doc_.count(name) == 1) {
            return doc_[name].get<double>();
        }
        else {
            return defaultValue;
        }
    }

    float getFloat(const std::string& name, float defaultValue) const
    {
        if (doc_.count(name) == 1) {
            return doc_[name].get<float>();
        }
        else {
            return defaultValue;
        }
    }

    bool getBool(const std::string& name, bool defaultValue) const
    {
        if (doc_.count(name) == 1) {
            return doc_[name].get<bool>();
        }
        else {
            return defaultValue;
        }
    }

    bool hasKey(const std::string& key) const
    {
        return doc_.find(key) != doc_.end();
    }

    int getInt(const std::string& name, int defaultValue) const
    {
        if (doc_.count(name) == 1) {
            return doc_[name].get<int>();
        }
        else {
            return defaultValue;
        }
    }

    bool setString(const std::string& name, std::string value)
    {
        if (doc_.count(name) != 1 || doc_[name].type() != nlohmann::detail::value_t::string || doc_[name] != value) {
            doc_[name] = value;
            return true;
        }
        return false;
    }
    bool setDouble(const std::string& name, double value)
    {
        if (doc_.count(name) != 1 || doc_[name].type() != nlohmann::detail::value_t::number_float || static_cast<double>(doc_[name]) != value) {
            doc_[name] = value;
            return true;
        }
        return false;
    }
    bool setBool(const std::string& name, bool value)
    {
        if (doc_.count(name) != 1 || doc_[name].type() != nlohmann::detail::value_t::boolean || static_cast<bool>(doc_[name]) != value) {
            doc_[name] = value;
            return true;
        }
        return false;
    }
    bool setInt(const std::string& name, int value)
    {
        if (doc_.count(name) != 1 || doc_[name].type() != nlohmann::detail::value_t::number_integer || static_cast<int>(doc_[name]) != value) {
            doc_[name] = value;
            return true;
        }
        return false;
    }

    void setChild(const std::string& name, Settings& value)
    {
        doc_[name] = value.doc_;
    }
};

}} //namespace
#endif
