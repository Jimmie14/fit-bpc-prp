#pragma once

#include <atomic>
#include <functional>
#include <thread>

namespace Manhattan::Core {
class App;

class TcpServer {

public:
    explicit TcpServer(int port);
    ~TcpServer();

    void Start();
    void Stop();

    void SetDataReceivedCallback(const std::function<void(const std::vector<uint8_t>&)>& callback)
    {
        _dataReceivedCallback = callback;
    }
    void ResetDataReceivedCallback()
    {
        _dataReceivedCallback = nullptr;
    }

private:
    void ServerLoop() const;

    int _serverSocket;
    int _port;
    std::atomic<bool> _running;
    std::thread _serverThread;

    std::function<void(const std::vector<uint8_t>&)> _dataReceivedCallback;
};
} // namespace Manhattan::Core
