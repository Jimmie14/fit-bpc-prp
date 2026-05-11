#pragma once

#include <any>
#include <string>
#include <typeindex>
#include <unordered_map>

namespace Manhattan::common {

using namespace std;

class Attributes {
public:
    Attributes() = default;


    template <typename T>
    void set(const std::string& key, T value)
    {
        _attributes[key] = Box(typeid(T), value);
    }

    template <typename T>
    bool has(const std::string& key) const
    {
        const auto it = _attributes.find(key);
        if (it == _attributes.end()) return false;

        return it->second.type == typeid(T);
    }

    template <typename T>
    T get(const std::string& key)
    {
        return _attributes[key];
    }

    void remove(const std::string& key)
    {
        _attributes.erase(key);
    }

private:
    struct Box {
        type_index type;
        any value;
    };

    unordered_map<string, Box> _attributes;
};
}