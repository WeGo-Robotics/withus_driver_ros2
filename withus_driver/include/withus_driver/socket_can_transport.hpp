#pragma once

#include <string>
#include "withus_driver/can_frame.hpp"

namespace withus_driver
{
enum class ReceiveResult
{
    Received,
    Timeout,
    Error
};

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

    ReceiveResult receive(CanFrame& frame, int timeout_ms);

private:
    int socket_fd_{-1};
};
} // namespace withus_driver
