#include "withus_driver/socket_can_transport.hpp"

#include <unistd.h>
#include <linux/can.h> // sockaddr_can
#include <linux/can/raw.h> // CAN_RAW
#include <net/if.h> // if_nametoindexm, IFNAMESIZ
#include <sys/socket.h> //socket, bind
#include <poll.h> // poll, pollfd
#include <algorithm> // std::copy_n

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

ReceiveResult SocketCanTransport::receive(
    CanFrame& frame, int timeout_ms)
{
    if(!is_open() || timeout_ms < 0){
        return ReceiveResult::Error;
    }

    pollfd poll_fd{};
    poll_fd.fd = socket_fd_;
    poll_fd.events = POLLIN;

    const int result = ::poll(&poll_fd, 1, timeout_ms);

    /*result check*/
    if(result == 0)
        return ReceiveResult::Timeout;
    if(result < 0)
        return ReceiveResult::Error;

    /*event check */
    //check the socket error or end event
    if((poll_fd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0)
        return ReceiveResult::Error;
    // check the data available event
    if((poll_fd.revents & POLLIN) == 0)
        return ReceiveResult::Error;

    /* read the frame */
    can_frame raw_frame{};

    // read the frame from the socket
    const ssize_t received_bytes = ::recv(
        socket_fd_,
        &raw_frame,
        sizeof(raw_frame),
        MSG_DONTWAIT
    );

    if(received_bytes < 0)
        return ReceiveResult::Error;
    // Linux CAN 프레임 전체를 받았는지 확인
    if(received_bytes != static_cast<ssize_t>(sizeof(raw_frame)))
        return ReceiveResult::Error;
    // Classical CAN 데이터 길이 검사
    if(raw_frame.len > CAN_MAX_DLEN)
        return ReceiveResult::Error;

    // convert the raw frame to CanFrame
    CanFrame received_frame{};

    received_frame.is_extended = (raw_frame.can_id & CAN_EFF_FLAG) != 0;
    received_frame.is_remote = (raw_frame.can_id & CAN_RTR_FLAG) != 0;
    received_frame.is_error = (raw_frame.can_id & CAN_ERR_FLAG) != 0;

    // 플래그를 제외한 ID 추출
    if(received_frame.is_error)
        received_frame.id = raw_frame.can_id & CAN_ERR_MASK;
    else if(received_frame.is_extended)
        received_frame.id = raw_frame.can_id & CAN_EFF_MASK;
    else
        received_frame.id = raw_frame.can_id & CAN_SFF_MASK;

    received_frame.length = raw_frame.len;

    // Remote 프레임에는 데이터 payload가 없음
    if(!received_frame.is_remote)
    {
        std::copy_n(
            raw_frame.data,
            received_frame.length,
            received_frame.data.begin()
        );
    }

    frame = received_frame;
    return ReceiveResult::Received;
}


SocketCanTransport::~SocketCanTransport()
{
    close();
}
}
