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
        config._view = &config._root;
        config._path.clear();
        return config;
    }

    Config operator[](const std::string& key) const
    {
        Config config;
        config._root = _root;
        config._view = _view;

        config._path = joinPath(_path, key);

        return config;
    }

    [[nodiscard]] bool exists() const
    {
        return _root.at_path(_path).node() != nullptr;
    }

    template <typename T>
    T value() const
    {
        const auto result = _root.at_path(_path).value<T>();
        if (!result.has_value()) {
            throw std::runtime_error("Missing config entry: " + _path);
        }
        return result.value();
    }

    [[nodiscard]] size_t size() const
    {
        const auto node = _root.at_path(_path);
        if (auto arr = node.as_array()) {
            return arr->size();
        }
        throw std::runtime_error("Config entry is not an array: " + _path);
    }

    [[nodiscard]] Config at(size_t index) const
    {
        Config config;
        config._root = _root;

        config._path = _path + "[" + std::to_string(index) + "]";
        return config;
    }

    struct Iterator {
        const Config* cfg;
        size_t index;

        Config operator*() const { return cfg->at(index); }
        bool operator!=(const Iterator& other) const { return index != other.index; }
        void operator++() { ++index; }
    };

    [[nodiscard]] Iterator begin() const { return {this, 0}; }
    [[nodiscard]] Iterator end() const { return {this, size()}; }

private:
    toml::table _root;
    std::string _path;

    const toml::node* _view = nullptr;

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