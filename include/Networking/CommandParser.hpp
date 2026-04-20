#pragma once
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace Manhattan::Core {
class CommandParser {
public:
    template <typename T>
    static std::optional<T> ParseValue(const std::string& key, const std::vector<std::string>& values)
    {
        for (const auto& val : values) {
            auto pos = val.find('=');
            if (pos != std::string::npos && val.substr(0, pos) == key) {
                std::istringstream iss(val.substr(pos + 1));
                T result;
                if (iss >> result) {
                    return result;
                }

                throw std::runtime_error("Failed to parse value for key: " + key);
            }
        }
        return std::nullopt; // Key not found
    }
};
} // namespace Manhattan::Core
