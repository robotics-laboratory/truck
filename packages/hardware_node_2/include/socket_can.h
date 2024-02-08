#pragma once

#include <functional>
#include <utility>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/eventfd.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <cstring>

class SocketInterface {
  public:
    SocketInterface() {
        s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        strcpy(ifr.ifr_name, "vxcan1");
        ioctl(s, SIOCGIFINDEX, &ifr);
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        bind(s, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr));
    }

    bool sendCanFrame(const can_frame& frame) {
        ssize_t nbytes = write(s, &frame, sizeof(frame));
        return (nbytes != -1);
    }

    bool processReceivedFrame(can_frame& frame) {
        ssize_t nbytes = read(s, &frame, sizeof(frame));
        return (nbytes != -1);
    }

  private:
    int s;
    struct ifreq ifr;
    struct sockaddr_can addr;
};
