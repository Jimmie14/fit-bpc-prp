#pragma once

#include <boost/property_tree/ptree.hpp>
#include <toml++/toml.hpp>

namespace Manhattan::config {

using namespace boost::property_tree;

class Config {
public:
    Config() = default;

    static Config load(const std::string& path)
    {
        Config config;

        config._root = toml::parse_file(path);

        return config;
    }

    Config operator[](const std::string& key) const
    {
        Config config;

        config._root = _root;
        config._path = joinPath(_path, key);

        return config;
    }

    template <typename T>
    T value() const
    {
        const auto result = _root.at_path(_path).value<T>();
        if (result.has_value()) return result.value();

        throw std::runtime_error("Config entry `" + _path + "` does not exist.");
    }

private:
    toml::table _root;
    std::string _path;

    static std::string joinPath(const std::string& left, const std::string& right)
    {
        if (left.empty()) return right;
        if (right.empty()) return left;
        return left + "." + right;
    }
};

struct Configurable {
    virtual void configure(const Config& config) = 0;

    virtual ~Configurable() = default;
};

}