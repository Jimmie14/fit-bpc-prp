#pragma once

#include <thread>
#include <atomic>

namespace Manhattan::Core
{
    class App;

    class TcpServer
    {
        const App& _app;
    public:
        explicit TcpServer(int port, const App& app);
        ~TcpServer();

        void Start();
        void Stop();

    private:
        void ServerLoop();

        int _serverSocket;
        int _port;
        std::atomic<bool> _running;
        std::thread _serverThread;
    };
}

