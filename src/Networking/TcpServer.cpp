#include "Networking/TcpServer.h"
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include "App.h"

namespace Manhattan::Core
{
    TcpServer::TcpServer(int port, const App& app) : _app(app), _serverSocket(-1), _port(port), _running(false)
    {
    }

    TcpServer::~TcpServer() {
        Stop();
    }

    void TcpServer::Start() {
        if (_running) return;

        _serverSocket = socket(AF_INET, SOCK_STREAM, 0);
        if (_serverSocket < 0) {
            std::cerr << "Error creating socket" << std::endl;
            return;
        }

        int opt = 1;
        if (setsockopt(_serverSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
            perror("setsockopt");
            close(_serverSocket);
            return;
        }

        struct sockaddr_in address;
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(_port);

        if (bind(_serverSocket, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
            perror("bind failed");
            close(_serverSocket);
            return;
        }

        if (listen(_serverSocket, 3) < 0) {
            perror("listen");
            close(_serverSocket);
            return;
        }

        _running = true;
        _serverThread = std::thread(&TcpServer::ServerLoop, this);
        std::cout << "TCP Server started on port " << _port << std::endl;
    }

    void TcpServer::Stop() {
        _running = false;
        if (_serverThread.joinable()) {
            _serverThread.join();
        }
        if (_serverSocket != -1) {
            close(_serverSocket);
            _serverSocket = -1;
        }
    }

    void TcpServer::ServerLoop() {
        while (_running) {
            struct sockaddr_in address;
            int addrlen = sizeof(address);

            // Use select to implement non-blocking accept or timeout
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(_serverSocket, &readfds);

            struct timeval timeout;
            timeout.tv_sec = 1;
            timeout.tv_usec = 0;

            int activity = select(_serverSocket + 1, &readfds, nullptr, nullptr, &timeout);

            if (activity < 0 && errno != EINTR) {
                std::cerr << "Select error" << std::endl;
                break;
            }

            if (activity > 0 && FD_ISSET(_serverSocket, &readfds)) {
                int new_socket = accept(_serverSocket, (struct sockaddr*)&address, (socklen_t*)&addrlen);
                if (new_socket < 0) {
                    if (_running) perror("accept");
                    continue;
                }

                // Keep connection open and read multiple messages
                while (_running) {
                    char buffer[1024] = {0};
                    ssize_t valRead = read(new_socket, buffer, sizeof(buffer));
                    if (valRead > 0) {
                        std::string message(buffer, valRead);
                        _app.ReceiveMessage(message);
                    } else if (valRead == 0) {
                        // Client closed connection
                        break;
                    } else {
                        // Error or interrupted
                        perror("read");
                        break;
                    }
                }
                close(new_socket);
            }
        }
    }
}

