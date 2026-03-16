#pragma once

#include <string>
#include <thread>
#include <atomic>
#include <netinet/in.h>

namespace Manhattan::Core
{
    class TcpServer
    {
    public:
        explicit TcpServer(int port);
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

