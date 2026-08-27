//
// Copyright (c) 2022 ZettaScale Technology
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
// which is available at https://www.apache.org/licenses/LICENSE-2.0.
//
// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
//
// Contributors:
//   ZettaScale Zenoh Team, <zenoh@zettascale.tech>

#include <arpa/inet.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>

#include "zenoh-pico/collections/string.h"
#include "zenoh-pico/config.h"
#include "zenoh-pico/link/transport/socket.h"
#include "zenoh-pico/system/platform.h"
#include "zenoh-pico/utils/logging.h"

// After the zenoh includes, so the Z_FEATURE_LINK_*
// macros the guards below test are already defined.
#if Z_FEATURE_LINK_CAN == 1
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#endif
#if Z_FEATURE_LINK_CAN == 1
#include "zenoh-pico/link/transport/can.h"
#endif
#include "zenoh-pico/utils/pointers.h"

z_result_t _z_socket_set_blocking(const _z_sys_net_socket_t *sock, bool blocking) {
    int flags = fcntl(sock->_fd, F_GETFL, 0);
    if (flags == -1) {
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }
    if (fcntl(sock->_fd, F_SETFL, blocking ? (flags & ~O_NONBLOCK) : (flags | O_NONBLOCK)) == -1) {
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }
    return _Z_RES_OK;
}

void _z_socket_close(_z_sys_net_socket_t *sock) {
    if (sock->_fd >= 0) {
        shutdown(sock->_fd, SHUT_RDWR);
        close(sock->_fd);
        sock->_fd = -1;
    }
}

z_result_t _z_socket_wait_readable(_z_socket_wait_iter_t *iter, uint32_t timeout_ms) {
    fd_set read_fds;
    int max_fd = 0;
    bool has_sockets = false;

    FD_ZERO(&read_fds);

    _z_socket_wait_iter_reset(iter);
    while (_z_socket_wait_iter_next(iter)) {
        const _z_sys_net_socket_t *sock = _z_socket_wait_iter_get_socket(iter);
        _z_socket_wait_iter_set_ready(iter, false);
        FD_SET(sock->_fd, &read_fds);
        if (sock->_fd > max_fd) {
            max_fd = sock->_fd;
        }
        has_sockets = true;
    }

    if (!has_sockets) {
        return _Z_RES_OK;
    }

    struct timeval timeout = {
        .tv_sec = (time_t)(timeout_ms / 1000U),
        .tv_usec = (suseconds_t)((timeout_ms % 1000U) * 1000U),
    };
    int result = select(max_fd + 1, &read_fds, NULL, NULL, &timeout);
    if (result < 0) {
        _Z_DEBUG("Errno: %d\n", errno);
        _Z_ERROR_RETURN(_Z_ERR_GENERIC);
    }

    bool has_data = false;
    _z_socket_wait_iter_reset(iter);
    while (_z_socket_wait_iter_next(iter)) {
        const _z_sys_net_socket_t *sock = _z_socket_wait_iter_get_socket(iter);
        bool is_ready = FD_ISSET(sock->_fd, &read_fds);
        _z_socket_wait_iter_set_ready(iter, is_ready);
        has_data |= is_ready;
    }

    return has_data ? _Z_RES_OK : _Z_NO_DATA_PROCESSED;
}

#if Z_FEATURE_LINK_BLUETOOTH == 1
#error "Bluetooth not supported yet on Unix port of Zenoh-Pico"
#endif

#if Z_FEATURE_LINK_CAN == 1

z_result_t _z_can_dev_open(_z_can_socket_t *sock, const char *dev, uint32_t bitrate, uint32_t dbitrate,
                           uint8_t rx_dst) {
    // Bit rates are set out of band on Linux (`ip link set can0 type can
    // bitrate ...`), and a virtual interface has none at all, so they are advisory.
    _ZP_UNUSED(bitrate);

    int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
        _Z_ERROR("CAN: socket(PF_CAN) failed: %d", errno);
        return _Z_ERR_GENERIC;
    }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    (void)strncpy(ifr.ifr_name, dev, sizeof(ifr.ifr_name) - 1);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        _Z_ERROR("CAN: no such interface '%s': %d", dev, errno);
        close(fd);
        return _Z_ERR_GENERIC;
    }

    // Let the kernel drop everything not addressed to us. CAN_EFF_FLAG in the
    // mask excludes standard-id frames, which are not ours.
    struct can_filter filter;
    filter.can_id = _Z_CAN_FILTER_ID(rx_dst) | CAN_EFF_FLAG;
    filter.can_mask = _Z_CAN_FILTER_MASK | CAN_EFF_FLAG;
    if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
        _Z_ERROR("CAN: CAN_RAW_FILTER failed: %d", errno);
        close(fd);
        return _Z_ERR_GENERIC;
    }

    // Ask for CAN FD. If the interface does not support it we fall back to
    // classic framing rather than failing, and report the mode we got.
    _Bool fd_mode = false;
    if (dbitrate != 0u) {
        int enable = 1;
        if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable)) == 0) {
            fd_mode = true;
        } else {
            _Z_DEBUG("CAN: interface '%s' has no CAN FD, using classic frames", dev);
        }
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        _Z_ERROR("CAN: bind('%s') failed: %d", dev, errno);
        close(fd);
        return _Z_ERR_GENERIC;
    }

    sock->_sock._fd = fd;
    sock->_fd_mode = fd_mode;
    return _Z_RES_OK;
}

void _z_can_dev_close(_z_can_socket_t *sock) {
    if (sock->_sock._fd >= 0) {
        close(sock->_sock._fd);
        sock->_sock._fd = -1;
    }
}

z_result_t _z_can_dev_send(const _z_can_socket_t *sock, uint32_t id, const uint8_t *data, size_t len) {
    struct canfd_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = id | CAN_EFF_FLAG;
    frame.len = (uint8_t)len;
    if (sock->_fd_mode) {
        frame.flags = CANFD_BRS;  // use the fast data phase when configured
    }
    memcpy(frame.data, data, len);

    size_t wire = sock->_fd_mode ? CANFD_MTU : CAN_MTU;
    ssize_t wb;
    do {
        wb = write(sock->_sock._fd, &frame, wire);
    } while ((wb < 0) && (errno == EINTR));
    if ((wb < 0) || ((size_t)wb != wire)) {
        _Z_ERROR("CAN: write failed: %d", errno);
        return _Z_ERR_GENERIC;
    }
    return _Z_RES_OK;
}

z_result_t _z_can_dev_recv(const _z_can_socket_t *sock, uint32_t *id, uint8_t *data, size_t *len) {
    struct canfd_frame frame;
    for (;;) {
        ssize_t rb = read(sock->_sock._fd, &frame, sizeof(frame));
        if (rb < 0) {
            if (errno == EINTR) {
                continue;
            }
            return _Z_ERR_GENERIC;
        }
        if (((rb != (ssize_t)CANFD_MTU) && (rb != (ssize_t)CAN_MTU)) || ((frame.can_id & CAN_EFF_FLAG) == 0u) ||
            ((frame.can_id & (CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0u) || (frame.len > *len)) {
            continue;  // runt, standard-id, remote or error frame
        }
        *id = frame.can_id & CAN_EFF_MASK;
        *len = frame.len;
        memcpy(data, frame.data, frame.len);
        return _Z_RES_OK;
    }
}

#endif  // Z_FEATURE_LINK_CAN == 1
