#include <iostream>
#include <string>

#include "withus_driver/socket_can_transport.hpp"

int main()
{
    const std::string interface_name = "vcan0";

    withus_driver::SocketCanTransport transport;

    if (!transport.open(interface_name)) {
        std::cerr << "Failed to open CAN interface: "
                  << interface_name << "\n";
        return 1;
    }

    withus_driver::CanFrame frame{};
    frame.id = 0x123;
    frame.length = 8;
    frame.data = {
        0x01, 0x02, 0x03, 0x04,
        0x05, 0x06, 0x07, 0x08
    };

    const auto result = transport.send(frame);

    if (result == withus_driver::SendResult::WouldBlock) {
        std::cerr << "CAN transmission is temporarily unavailable.\n";
        return 1;
    }

    if (result == withus_driver::SendResult::Error) {
        std::cerr << "Failed to send CAN frame.\n";
        return 1;
    }

    std::cout << "CAN frame accepted for transmission.\n";
    return 0;
}