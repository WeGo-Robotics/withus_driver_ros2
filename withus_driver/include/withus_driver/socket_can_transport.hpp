#pragma once

#include <string>

namespace withus_driver
{
class SocketCanTransport
{
public:
    SocketCanTransport()=default;
    ~SocketCanTransport();

    SocketCanTransport(const SocketCanTransport&) = delete;
    SocketCanTransport& operator=(const SocketCanTransport&) = delete;

    bool open(const std::string& interface_name);
    void close() noexcept;
    bool is_open() const noexcept;
private:
    int socket_fd_{-1};
};
} // namespace withus_driver