#pragma once

namespace Manhattan::Core {
class EventBus {
public:
    template <typename T>
    using Handler = std::function<void(const T&)>;

    template <typename T>
    void Subscribe(Handler<T> handler)
    {
        auto& vec = handlers[typeid(T)];

        vec.push_back([handler](const void* event) {
            handler(*static_cast<const T*>(event));
        });
    }

    template <typename T>
    void Publish(const T& event) const
    {
        const auto it = handlers.find(typeid(T));
        if (it == handlers.end())
            return;

        for (auto& h : it->second) {
            h(&event);
        }
    }

private:
    std::unordered_map<std::type_index,
        std::vector<std::function<void(const void*)>>>
        handlers;
};
}