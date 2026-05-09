#pragma once

#include "RosUnit.hpp"

#include "Config/Configurable.hpp"
#include <filesystem>
#include <opencv2/core/utility.hpp>
#include <string>

namespace Manhattan::common {

using namespace Manhattan::config;
namespace fs = std::filesystem;

class ConfigManager : public Node {
public:

    explicit ConfigManager()
        : Node("config")
        , _configPath(declare_parameter("config", "manhattan.toml"))
    {
        _config = Config::load(_configPath);
        _lastWriteTime = fs::last_write_time(_configPath);

        _timer = create_wall_timer(100ms, [this] {
            checkConfig();
        });
    }

    template <typename T>
    requires std::is_base_of_v<Configurable, T> T get(const std::string& key)
    {
        auto config = T();

        config.configure(_config[key]);

        return config;
    }

    template <typename T>
    requires std::is_base_of_v<Configurable, T> void watch(const std::string& key, std::function<void(const T&)> callback)
    {
        lock_guard guard(_lock);

        _watchers.push_back([this, key, callback] {
            callback(get<T>(key));
        });

        callback(get<T>(key));
    }

private:
    TimerBase::SharedPtr _timer;

    std::string _configPath;
    Config _config;
    fs::file_time_type _lastWriteTime;

    std::mutex _lock;
    vector<std::function<void()>> _watchers;

    void checkConfig()
    {
        const auto currentWriteTime = fs::last_write_time(_configPath);
        if (currentWriteTime == _lastWriteTime) return;

        _lastWriteTime = currentWriteTime;

        lock_guard guard(_lock);

        try {
            _config = Config::load(_configPath);

            for (auto& watcher : _watchers) {
                watcher();
            }
        } catch (const std::exception& e) {
            std::cout << "Error in config watcher: " << e.what() << std::endl;
        }

    }
};
}
