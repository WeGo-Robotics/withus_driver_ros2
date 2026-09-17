#include "withus_driver/socket_can_transport.hpp"

#include <unistd.h>
#include <linux/can.h> // sockaddr_can
#include <linux/can/raw.h> // CAN_RAW
#include <net/if.h> // if_nametoindexm, IFNAMESIZ
#include <sys/socket.h> //socket, bind

namespace withus_driver
{
bool SocketCanTransport::open(const std::string& interface_name)
{
    // verify the interface name is valid and not already open
    if(is_open() || interface_name.empty() ||
                interface_name.size() >= IFNAMSIZ ||
                interface_name.find('\0') != std::string::npos)
    {
        return false;
    }

    // create the socket
    const int fd =  ::socket(PF_CAN, SOCK_RAW, CAN_RAW);

    if(fd < 0){
        return false;
    }

    // change the interface name to an index
    const unsigned int interface_index =
        ::if_nametoindex(interface_name.c_str());

    if(interface_index == 0){
        ::close(fd);
        return false;
    }

    // bind the socket to the interface
    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = static_cast<int>(interface_index);

    if(::bind(fd,
        reinterpret_cast<sockaddr*>(&address),
        sizeof(address)) < 0)
    {
        ::close(fd);
        return false;
    }

    socket_fd_ = fd;
    return true;

}

bool SocketCanTransport::is_open() const noexcept
{
    return socket_fd_ >=0;
}

void SocketCanTransport::close() noexcept
{
    if(!is_open()){
        return;
    }
    ::close(socket_fd_);
    socket_fd_ = -1;
}

SocketCanTransport::~SocketCanTransport()
{
    close();
}
}