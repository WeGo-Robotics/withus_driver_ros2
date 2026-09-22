#include "withus_driver/can_interface.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <linux/if_link.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

namespace withus_driver
{
namespace
{
class SocketOwner
{
public:
    explicit SocketOwner(int fd) : fd_(fd) {}
    ~SocketOwner() { if (fd_ >= 0) ::close(fd_); }
    SocketOwner(const SocketOwner&) = delete;
    SocketOwner& operator=(const SocketOwner&) = delete;
    int get() const noexcept { return fd_; }

private:
    int fd_;
};

// Validate all attributes, including attributes after the one being sought.
// Copy headers rather than assuming alignment of received byte storage.
bool read_kind(const unsigned char* data, std::size_t size, bool& is_vcan)
{
    bool found_kind = false;
    while (size > 0) {
        if (size < sizeof(rtattr)) return false;
        rtattr attribute{};
        std::memcpy(&attribute, data, sizeof(attribute));
        const std::size_t length = attribute.rta_len;
        if (length < sizeof(rtattr) || length > size) return false;
        const std::size_t step = RTA_ALIGN(length);
        if (step > size) return false;
        if ((attribute.rta_type & NLA_TYPE_MASK) == IFLA_INFO_KIND) {
            if (found_kind) return false;
            found_kind = true;
            constexpr char expected[] = "vcan";
            is_vcan = length - sizeof(rtattr) == sizeof(expected) &&
                std::memcmp(data + sizeof(rtattr), expected, sizeof(expected)) == 0;
        }
        data += step;
        size -= step;
    }
    return found_kind;
}

bool read_link_info(const unsigned char* data, std::size_t size)
{
    bool found_info = false;
    bool is_vcan = false;
    while (size > 0) {
        if (size < sizeof(rtattr)) return false;
        rtattr attribute{};
        std::memcpy(&attribute, data, sizeof(attribute));
        const std::size_t length = attribute.rta_len;
        if (length < sizeof(rtattr) || length > size) return false;
        const std::size_t step = RTA_ALIGN(length);
        if (step > size) return false;
        if ((attribute.rta_type & NLA_TYPE_MASK) == IFLA_LINKINFO) {
            if (found_info) return false;
            found_info = true;
            if (!read_kind(data + sizeof(rtattr), length - sizeof(rtattr), is_vcan)) {
                return false;
            }
        }
        data += step;
        size -= step;
    }
    return found_info && is_vcan;
}
}  // namespace

bool is_vcan_interface(const std::string& interface_name)
{
    if (interface_name.empty() || interface_name.size() >= IFNAMSIZ ||
        interface_name.find('\0') != std::string::npos) return false;
    const unsigned int index = ::if_nametoindex(interface_name.c_str());
    if (index == 0) return false;

    SocketOwner socket(::socket(AF_NETLINK, SOCK_RAW | SOCK_CLOEXEC, NETLINK_ROUTE));
    if (socket.get() < 0) return false;
    // One response only: no unbounded dump/retry loop on malformed or missing data.
    const timeval timeout{0, 250000};
    if (::setsockopt(socket.get(), SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        return false;
    }
    sockaddr_nl local{};
    local.nl_family = AF_NETLINK;
    if (::bind(socket.get(), reinterpret_cast<const sockaddr*>(&local), sizeof(local)) < 0) {
        return false;
    }
    socklen_t local_size = sizeof(local);
    if (::getsockname(socket.get(), reinterpret_cast<sockaddr*>(&local), &local_size) < 0 ||
        local_size != sizeof(local) || local.nl_family != AF_NETLINK || local.nl_pid == 0) {
        return false;
    }

    struct Request
    {
        nlmsghdr header;
        ifinfomsg link;
    } request{};
    request.header.nlmsg_len = NLMSG_LENGTH(sizeof(ifinfomsg));
    request.header.nlmsg_type = RTM_GETLINK;
    request.header.nlmsg_flags = NLM_F_REQUEST;
    request.header.nlmsg_seq = 1;
    request.header.nlmsg_pid = local.nl_pid;
    request.link.ifi_family = AF_UNSPEC;
    request.link.ifi_index = static_cast<int>(index);
    sockaddr_nl kernel{};
    kernel.nl_family = AF_NETLINK;
    const auto sent = ::sendto(socket.get(), &request, request.header.nlmsg_len, MSG_DONTWAIT,
        reinterpret_cast<const sockaddr*>(&kernel), sizeof(kernel));
    if (sent != static_cast<ssize_t>(request.header.nlmsg_len)) return false;

    std::array<unsigned char, 16384> buffer{};
    sockaddr_nl sender{};
    iovec vector{buffer.data(), buffer.size()};
    msghdr message{};
    message.msg_name = &sender;
    message.msg_namelen = sizeof(sender);
    message.msg_iov = &vector;
    message.msg_iovlen = 1;
    const auto received = ::recvmsg(socket.get(), &message, 0);
    if (received < static_cast<ssize_t>(NLMSG_LENGTH(sizeof(ifinfomsg))) ||
        (message.msg_flags & (MSG_TRUNC | MSG_CTRUNC)) != 0 ||
        message.msg_namelen != sizeof(sender) || sender.nl_family != AF_NETLINK ||
        sender.nl_pid != 0 || sender.nl_groups != 0) return false;

    nlmsghdr header{};
    std::memcpy(&header, buffer.data(), sizeof(header));
    if (header.nlmsg_type != RTM_NEWLINK || header.nlmsg_seq != request.header.nlmsg_seq ||
        header.nlmsg_pid != local.nl_pid ||
        (header.nlmsg_flags & (NLM_F_MULTI | NLM_F_DUMP_INTR)) != 0 ||
        header.nlmsg_len < NLMSG_LENGTH(sizeof(ifinfomsg)) ||
        header.nlmsg_len > static_cast<std::size_t>(received) ||
        NLMSG_ALIGN(header.nlmsg_len) != static_cast<std::size_t>(received)) return false;

    ifinfomsg link{};
    std::memcpy(&link, buffer.data() + NLMSG_HDRLEN, sizeof(link));
    if (link.ifi_index != static_cast<int>(index)) return false;
    constexpr std::size_t attributes_offset = NLMSG_LENGTH(sizeof(ifinfomsg));
    return read_link_info(buffer.data() + attributes_offset,
        header.nlmsg_len - attributes_offset);
}

}  // namespace withus_driver
