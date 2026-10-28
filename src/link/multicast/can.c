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
//

#include "zenoh-pico/link/config/can.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "zenoh-pico/config.h"
#include "zenoh-pico/link/manager.h"
#include "zenoh-pico/link/transport/can.h"
#include "zenoh-pico/utils/logging.h"

#if Z_FEATURE_LINK_CAN == 1

#define _Z_CAN_DEV_MAX 32

z_result_t _z_endpoint_can_valid(_z_endpoint_t *endpoint) {
    z_result_t ret = _Z_RES_OK;

    _z_string_t can_str = _z_string_alias_str(CAN_SCHEMA);
    if (!_z_string_equals(&endpoint->_locator._protocol, &can_str)) {
        _Z_ERROR_LOG(_Z_ERR_CONFIG_LOCATOR_INVALID);
        ret = _Z_ERR_CONFIG_LOCATOR_INVALID;
    }

    if (ret == _Z_RES_OK) {
        size_t addr_len = _z_string_len(&endpoint->_locator._address);
        // The address is the interface name: "can0", "vcan0", or a Zephyr
        // devicetree label. Empty is meaningless and an over-long one would
        // silently truncate into a different interface, so reject both here
        // rather than at open() where the error is less obvious.
        if ((addr_len == 0) || (addr_len >= _Z_CAN_DEV_MAX)) {
            _Z_ERROR_LOG(_Z_ERR_CONFIG_LOCATOR_INVALID);
            ret = _Z_ERR_CONFIG_LOCATOR_INVALID;
        }
    }

    return ret;
}

// Read one unsigned integer config value, accepting decimal and 0x-prefixed
// hex -- identifiers are conventionally written in hex, bit rates in decimal,
// and forcing one notation on both would make every endpoint harder to read.
static uint32_t __z_can_cfg_u32(const _z_str_intmap_t *cfg, uint8_t key, uint32_t dflt) {
    char *s = _z_str_intmap_get(cfg, key);
    if (s == NULL) {
        return dflt;
    }

    uint32_t base = 10;
    const char *p = s;
    if ((p[0] == '0') && ((p[1] == 'x') || (p[1] == 'X'))) {
        base = 16;
        p += 2;
    }

    uint32_t acc = 0;
    _Bool any = false;
    for (; *p != '\0'; p++) {
        uint32_t digit;
        if ((*p >= '0') && (*p <= '9')) {
            digit = (uint32_t)(*p - '0');
        } else if ((base == 16) && (*p >= 'a') && (*p <= 'f')) {
            digit = (uint32_t)(*p - 'a') + 10u;
        } else if ((base == 16) && (*p >= 'A') && (*p <= 'F')) {
            digit = (uint32_t)(*p - 'A') + 10u;
        } else {
            // Malformed: fall back to the default rather than half-parsing.
            return dflt;
        }
        if (acc > ((UINT32_MAX - digit) / base)) {
            return dflt;  // overflow
        }
        acc = (acc * base) + digit;
        any = true;
    }

    return any ? acc : dflt;
}

static z_result_t __z_can_open(_z_link_t *self, _Bool unicast) {
    char dev[_Z_CAN_DEV_MAX];
    size_t addr_len = _z_string_len(&self->_endpoint._locator._address);
    if (addr_len >= sizeof(dev)) {
        return _Z_ERR_CONFIG_LOCATOR_INVALID;
    }
    memcpy(dev, _z_string_data(&self->_endpoint._locator._address), addr_len);
    dev[addr_len] = '\0';

    const _z_str_intmap_t *cfg = &self->_endpoint._config;
    uint32_t bitrate = __z_can_cfg_u32(cfg, CAN_CONFIG_BITRATE_KEY, CAN_CONFIG_DEFAULT_BITRATE);
    uint32_t dbitrate = __z_can_cfg_u32(cfg, CAN_CONFIG_DBITRATE_KEY, CAN_CONFIG_DEFAULT_DBITRATE);
    _Bool has_id = _z_str_intmap_get(cfg, CAN_CONFIG_ID_KEY) != NULL;
    uint32_t node = __z_can_cfg_u32(cfg, CAN_CONFIG_ID_KEY, CAN_CONFIG_DEFAULT_ID);
    uint32_t peer = __z_can_cfg_u32(cfg, CAN_CONFIG_PEER_KEY, _Z_CAN_NODE_NONE);

    // 0xFF is the broadcast address and 0xFE the unassigned one, so neither can
    // name a node.
    if (has_id && (node > _Z_CAN_NODE_MAX)) {
        _Z_ERROR("CAN: node id %u is out of range 0..%u", (unsigned)node, (unsigned)_Z_CAN_NODE_MAX);
        return _Z_ERR_CONFIG_LOCATOR_INVALID;
    }
    if (unicast) {
        if ((peer > _Z_CAN_NODE_MAX) || (has_id && (peer == node))) {
            _Z_ERROR("CAN: a unicast endpoint needs peer=<0..%u>, other than its own id", (unsigned)_Z_CAN_NODE_MAX);
            return _Z_ERR_CONFIG_LOCATOR_INVALID;
        }
        if (!has_id) {
            // No id: ask the node we are connecting to for one
            uint8_t assigned = 0;
            z_result_t claimed = _z_can_claim_id(dev, bitrate, dbitrate, (uint8_t)peer, &assigned);
            if (claimed != _Z_RES_OK) {
                return claimed;
            }
            node = assigned;
        }
    } else {
        peer = _Z_CAN_NODE_NONE;
    }

    z_result_t ret = _z_open_can(&self->_socket._can, dev, bitrate, dbitrate, (uint8_t)node, (uint8_t)peer);
    if (ret != _Z_RES_OK) {
        return ret;
    }
    self->_mtu = _Z_CAN_MTU_SIZE;
    return _Z_RES_OK;
}

// A link that dials out is unicast, and one that listens is multicast, the way
// `udp` splits them. `_z_open_link` runs the first and `_z_listen_link` the
// second, so one endpoint syntax serves both and `peer` is what makes it a pair.
z_result_t _z_f_link_open_can(_z_link_t *self) { return __z_can_open(self, true); }

z_result_t _z_f_link_listen_can(_z_link_t *self) { return __z_can_open(self, false); }

void _z_f_link_close_can(_z_link_t *self) { _z_close_can(&self->_socket._can); }

void _z_f_link_free_can(_z_link_t *self) { _ZP_UNUSED(self); }

// The read path holds reassembly state in the socket, so these cast the const
// away. Reads come from one thread and writes from another; they touch
// disjoint fields (`_rx` vs `_tid`).
size_t _z_f_link_write_can(const _z_link_t *self, const uint8_t *ptr, size_t len, _z_sys_net_socket_t *socket) {
    _ZP_UNUSED(socket);
    return _z_send_can((_z_can_socket_t *)&self->_socket._can, ptr, len);
}

size_t _z_f_link_write_all_can(const _z_link_t *self, const uint8_t *ptr, size_t len) {
    // One datagram is segmented and sent whole, so there is no partial write to
    // loop over.
    return _z_send_can((_z_can_socket_t *)&self->_socket._can, ptr, len);
}

size_t _z_f_link_read_can(const _z_link_t *self, uint8_t *ptr, size_t len, _z_slice_t *addr) {
    return _z_read_can((_z_can_socket_t *)&self->_socket._can, ptr, len, addr);
}

size_t _z_f_link_read_exact_can(const _z_link_t *self, uint8_t *ptr, size_t len, _z_slice_t *addr,
                                _z_sys_net_socket_t *socket) {
    _ZP_UNUSED(socket);
    // "Exact" and "best-effort" collapse on a datagram link.
    return _z_read_can((_z_can_socket_t *)&self->_socket._can, ptr, len, addr);
}

size_t _z_f_link_read_socket_can(const _z_sys_net_socket_t socket, uint8_t *ptr, size_t len) {
    // Reassembly state lives in the link, which a bare socket read does not
    // have, so reaching here is a logic bug at the call site.
    _ZP_UNUSED(socket);
    _ZP_UNUSED(ptr);
    _ZP_UNUSED(len);
    _Z_ERROR("CAN: _z_f_link_read_socket_can must not be called");
    return SIZE_MAX;
}

uint16_t _z_get_link_mtu_can(void) { return _Z_CAN_MTU_SIZE; }

static z_result_t __z_new_link_can(_z_link_t *zl, _z_endpoint_t endpoint, uint8_t transport) {
    zl->_type = _Z_LINK_TYPE_CAN;
    zl->_cap._transport = transport;
    // The link segments and reassembles itself and always delivers whole
    // datagrams, so zenoh sees a datagram link with a 4095-byte MTU. The batch
    // size follows the MTU, which is what lets Join, InitAck and a query reply
    // cross in one batch, and zenoh's own fragmentation only runs above it.
    zl->_cap._flow = Z_LINK_CAP_FLOW_DATAGRAM;
    // Reliable per frame -- CRC, ACK slot, retransmission -- but not end to end:
    // controller buffers overrun and bus-off drops everything.
    zl->_cap._is_reliable = false;

    zl->_mtu = _z_get_link_mtu_can();
    zl->_endpoint = endpoint;

    zl->_open_f = _z_f_link_open_can;
    zl->_listen_f = _z_f_link_listen_can;
    zl->_close_f = _z_f_link_close_can;
    zl->_free_f = _z_f_link_free_can;

    zl->_write_f = _z_f_link_write_can;
    zl->_write_all_f = _z_f_link_write_all_can;
    zl->_read_f = _z_f_link_read_can;
    zl->_read_exact_f = _z_f_link_read_exact_can;
    zl->_read_socket_f = _z_f_link_read_socket_can;

    return _Z_RES_OK;
}

// A frame addressed to one node is unicast and one addressed to all of them is
// multicast; the wire format is the same, so these differ only in capability.
z_result_t _z_new_link_can_unicast(_z_link_t *zl, _z_endpoint_t endpoint) {
    return __z_new_link_can(zl, endpoint, Z_LINK_CAP_TRANSPORT_UNICAST);
}

z_result_t _z_new_link_can_multicast(_z_link_t *zl, _z_endpoint_t endpoint) {
    return __z_new_link_can(zl, endpoint, Z_LINK_CAP_TRANSPORT_MULTICAST);
}

// ---------------------------------------------------------------------------
// The datagram interface over the platform's raw frames.

z_result_t _z_open_can(_z_can_socket_t *sock, const char *dev, uint32_t bitrate, uint32_t dbitrate, uint8_t node,
                       uint8_t peer) {
    memset(sock, 0, sizeof(*sock));
    sock->_node = node;
    sock->_peer = peer;
    sock->_prio = _Z_CAN_PRIO_DEFAULT;
    // A unicast link hears frames addressed to this node, a multicast one the
    // frames addressed to everybody. The controller's filter does that cut, so
    // the CPU never sees the rest.
    uint8_t rx_dst = (peer == _Z_CAN_NODE_NONE) ? _Z_CAN_NODE_BROADCAST : node;
    return _z_can_dev_open(sock, dev, bitrate, dbitrate, rx_dst);
}

void _z_close_can(_z_can_socket_t *sock) {
    _z_can_dev_close(sock);
    _z_can_rx_clear(&sock->_rx);
}

static z_result_t __z_can_emit(void *ctx, uint32_t id, const uint8_t *data, size_t len) {
    return _z_can_dev_send((const _z_can_socket_t *)ctx, id, data, len);
}

// ---------------------------------------------------------------------------
// Asking for a node id

__attribute__((weak)) void _z_can_uid(uint8_t uid[_Z_CAN_UID_SIZE]) {
    for (size_t i = 0; i < _Z_CAN_UID_SIZE; i += sizeof(uint32_t)) {
        uint32_t r = z_random_u32();
        memcpy(&uid[i], &r, sizeof(r));
    }
}

// A few bits of the UID, for the key field of its request, so that two devices that
// ask at once send different identifiers. The router does not check it.
static uint8_t __z_can_uid_key(const uint8_t *uid) {
    uint8_t h = 0;
    for (size_t i = 0; i < _Z_CAN_UID_SIZE; i++) {
        h = (uint8_t)((uint8_t)(h << 3) | (uint8_t)(h >> 5)) ^ uid[i];
    }
    return (uint8_t)(h & _Z_CAN_KEY_MASK);
}

z_result_t _z_can_claim_id(const char *dev, uint32_t bitrate, uint32_t dbitrate, uint8_t peer, uint8_t *node) {
    _z_can_socket_t sock;
    memset(&sock, 0, sizeof(sock));
    sock._node = _Z_CAN_NODE_UNASSIGNED;
    sock._peer = peer;

    // Hear what is addressed to the unassigned address, which is where the router answers
    z_result_t ret = _z_can_dev_open(&sock, dev, bitrate, dbitrate, _Z_CAN_NODE_UNASSIGNED);
    if (ret != _Z_RES_OK) {
        return ret;
    }

    uint8_t uid[_Z_CAN_UID_SIZE];
    _z_can_uid(uid);
    uint32_t request = _Z_CAN_ID_MAKE_CTL(_Z_CAN_KIND_ID_REQUEST, __z_can_uid_key(uid), peer, _Z_CAN_NODE_UNASSIGNED);
    size_t frame_max = sock._fd_mode ? _Z_CAN_FD_FRAME_LEN : _Z_CAN_CLASSIC_FRAME_LEN;

    ret = _Z_ERR_TRANSPORT_OPEN_FAILED;
    for (uint8_t attempt = 0; (attempt < Z_CAN_ID_REQUEST_TRIES) && (ret != _Z_RES_OK); attempt++) {
        // Devices that start together would otherwise all ask at the same instant
        z_sleep_ms(z_random_u32() % Z_CAN_ID_REQUEST_JITTER_MS);
        if (_z_can_encode(uid, sizeof(uid), request, attempt, frame_max, __z_can_emit, &sock) != _Z_RES_OK) {
            continue;
        }

        z_clock_t start = z_clock_now();
        while (z_clock_elapsed_ms(&start) < Z_CAN_ID_REQUEST_WAIT_MS) {
            uint32_t id = 0;
            uint8_t frame[_Z_CAN_FD_FRAME_LEN];
            size_t flen = sizeof(frame);
            if (_z_can_dev_recv(&sock, &id, frame, &flen) != _Z_RES_OK) {
                continue;  // nothing within the read timeout
            }
            if ((_Z_CAN_ID_KIND(id) != _Z_CAN_KIND_ID_ASSIGN) || (_Z_CAN_ID_SRC(id) != peer)) {
                continue;
            }

            uint8_t answer[_Z_CAN_UID_SIZE + 1u];
            size_t alen = 0;
            if ((_z_can_decode(&sock._rx, id, frame, flen, answer, sizeof(answer), &alen) != 1) ||
                (alen != sizeof(answer)) || (memcmp(answer, uid, _Z_CAN_UID_SIZE) != 0)) {
                continue;  // not for us
            }
            if ((answer[_Z_CAN_UID_SIZE] > _Z_CAN_NODE_MAX) || (answer[_Z_CAN_UID_SIZE] == peer)) {
                _Z_ERROR("CAN: the node gave an id that cannot be used: %u", (unsigned)answer[_Z_CAN_UID_SIZE]);
                continue;
            }
            *node = answer[_Z_CAN_UID_SIZE];
            ret = _Z_RES_OK;
            break;
        }
    }

    _z_can_dev_close(&sock);
    _z_can_rx_clear(&sock._rx);
    if (ret != _Z_RES_OK) {
        _Z_ERROR("CAN: node %u did not give an id", (unsigned)peer);
    }
    return ret;
}

size_t _z_send_can(_z_can_socket_t *sock, const uint8_t *ptr, size_t len) {
    // Zenoh gives the link no per-batch priority yet, so everything goes out
    // at the socket's, and the key field stays 0 (see link/transport/can.h).
    uint8_t dst = (sock->_peer == _Z_CAN_NODE_NONE) ? _Z_CAN_NODE_BROADCAST : sock->_peer;
    uint32_t id = _Z_CAN_ID_MAKE(sock->_prio, 0u, dst, sock->_node);
    size_t frame_max = sock->_fd_mode ? _Z_CAN_FD_FRAME_LEN : _Z_CAN_CLASSIC_FRAME_LEN;
    uint8_t tid = sock->_tid++;

    if (_z_can_encode(ptr, len, id, tid, frame_max, __z_can_emit, sock) != _Z_RES_OK) {
        return SIZE_MAX;
    }
    return len;
}

size_t _z_read_can(_z_can_socket_t *sock, uint8_t *ptr, size_t len, _z_slice_t *addr) {
    uint8_t frame[_Z_CAN_FD_FRAME_LEN];

    // Loop until a datagram completes. Frames of several transfers interleave
    // on a bus, so one frame rarely finishes anything.
    for (;;) {
        uint32_t id = 0;
        size_t flen = sizeof(frame);
        if (_z_can_dev_recv(sock, &id, frame, &flen) != _Z_RES_OK) {
            return SIZE_MAX;
        }

        if (_Z_CAN_ID_KIND(id) != _Z_CAN_KIND_DATA) {
            continue;  // not data: a request for an id, or the answer to one
        }

        // The controller's filter keeps frames for other nodes out, but it is not the only thing that decides what
        // this link acts on: a filter outlives the task that set it, so one left by a task that was killed would
        // let this link hear, and answer, frames addressed to the id that task had.
        uint8_t want = (sock->_peer == _Z_CAN_NODE_NONE) ? _Z_CAN_NODE_BROADCAST : sock->_node;
        if (_Z_CAN_ID_DST(id) != want) {
            continue;
        }

        uint8_t src = _Z_CAN_ID_SRC(id);
        if (src == sock->_node) {
            continue;  // our own frame, echoed back by a loopback-enabled interface
        }
        if ((sock->_peer != _Z_CAN_NODE_NONE) && (src != sock->_peer)) {
            continue;  // unicast: someone else wrote to this node
        }

        size_t dlen = 0;
        if (_z_can_decode(&sock->_rx, id, frame, flen, ptr, len, &dlen) == 0) {
            continue;
        }

        if (addr != NULL) {
            assert(addr->len >= _Z_CAN_ADDR_SIZE);
            addr->len = _Z_CAN_ADDR_SIZE;
            *(uint8_t *)addr->start = src;
        }
        return dlen;
    }
}

#endif
