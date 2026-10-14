#include <iostream>
#include <string>
#include <cstddef>
#include <iomanip>

#include "withus_driver/socket_can_transport.hpp"

int main(int argc, char** argv)
{
    if(argc != 2){
        std::cerr << "Usage : can_connection_check <interface_name>\n";
        return 2;
    }

    const std::string interface_name = argv[1];
    std::cout << "Interface: " << interface_name << "\n";

    withus_driver::SocketCanTransport transport;
    if(!transport.open(interface_name)){
        std::cerr << "Failed to open CAN interface: "
                << interface_name << "\n";
        return 1;
    }

    std::cout << "CAN socket opened on: " << interface_name << "\n";


    withus_driver::CanFrame frame{};

    const auto result = transport.receive(frame, 1000);

    if(result == withus_driver::ReceiveResult::Timeout){
        std::cerr << "No CAN frame received within 1 sec.\n";
        return 1;
    }

    if(result == withus_driver::ReceiveResult::Error){
        std::cerr << "Failed to receive a CAN frame.\n";
        return 1;
    }

    std::cout << "Received CAN frame\n";
    std::cout << "ID: 0x" << std::hex << frame.id << std::dec << "\n";
    std::cout << "Length: " << static_cast<unsigned int>(frame.length)
              << "\n";

    if(frame.is_remote){
        std::cout << "Data: RTR (no payload)\n";
    }else{
        std::cout << "Data: " << std::hex << std::setfill('0');

        for(std::size_t i = 0; i < frame.length; ++i){
            std::cout << ' '
                      << std::setw(2)
                      << static_cast<unsigned int>(frame.data[i]);
        }

        std::cout << std::dec << std::setfill(' ') << "\n";
    }


    transport.close();
    if(transport.is_open()){
        std::cerr << "Transport is still marked as open.\n";
        return 1;
    }

    std::cout << "CAN socket closed.\n";
    return 0;
}
