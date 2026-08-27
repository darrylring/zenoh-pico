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

#ifndef ZENOH_PICO_SYSTEM_LINK_CAN_H
#define ZENOH_PICO_SYSTEM_LINK_CAN_H

#include <stdint.h>

#include "zenoh-pico/collections/slice.h"
#include "zenoh-pico/config.h"
#include "zenoh-pico/system/platform.h"
#include "zenoh-pico/utils/result.h"

#ifdef __cplusplus
extern "C" {
#endif

#if Z_FEATURE_LINK_CAN == 1

// ---------------------------------------------------------------------------
// Identifier
//
// One 29-bit extended identifier layout serves multicast and unicast. CAN
// arbitrates on the identifier -- lowest wins -- so what goes in the high bits
// is what the bus treats as most urgent, and the low bits are what a
// controller's hardware filter can select on.
//
//   28..26  priority   zenoh priority (1 = real-time ... 7 = background)
//   25..21  key id     the sender's ExprId for the topic, 0 = none
//   20..16  reserved   zero
//   15..8   destination node, 0xFF = every node (multicast)
//    7..0   source node
//
// A frame addressed to 0xFF is multicast; any other destination is unicast.
// The frame format is identical, only the destination differs.
//
// The key id field is reserved for topic filtering in hardware. The link does
// not parse zenoh's messages, so it does not fill it yet and sends 0.
#define _Z_CAN_ID_SRC_SHIFT 0u
#define _Z_CAN_ID_DST_SHIFT 8u
#define _Z_CAN_ID_KEY_SHIFT 21u
#define _Z_CAN_ID_PRIO_SHIFT 26u

#define _Z_CAN_NODE_MASK 0xFFu
#define _Z_CAN_NODE_BROADCAST 0xFFu  // destination of multicast frames
#define _Z_CAN_NODE_NONE 0xFFu       // "no peer": the unicast peer of a multicast link
#define _Z_CAN_KEY_MASK 0x1Fu
#define _Z_CAN_PRIO_MASK 0x07u

#define _Z_CAN_ID_MAKE(prio, key, dst, src)                                                                  \
    ((((uint32_t)(prio)&_Z_CAN_PRIO_MASK) << _Z_CAN_ID_PRIO_SHIFT) |                                         \
     (((uint32_t)(key)&_Z_CAN_KEY_MASK) << _Z_CAN_ID_KEY_SHIFT) |                                            \
     (((uint32_t)(dst)&_Z_CAN_NODE_MASK) << _Z_CAN_ID_DST_SHIFT) | (((uint32_t)(src)&_Z_CAN_NODE_MASK)))
#define _Z_CAN_ID_SRC(id) ((uint8_t)(((id) >> _Z_CAN_ID_SRC_SHIFT) & _Z_CAN_NODE_MASK))
#define _Z_CAN_ID_DST(id) ((uint8_t)(((id) >> _Z_CAN_ID_DST_SHIFT) & _Z_CAN_NODE_MASK))
#define _Z_CAN_ID_KEY(id) ((uint8_t)(((id) >> _Z_CAN_ID_KEY_SHIFT) & _Z_CAN_KEY_MASK))
#define _Z_CAN_ID_PRIO(id) ((uint8_t)(((id) >> _Z_CAN_ID_PRIO_SHIFT) & _Z_CAN_PRIO_MASK))

// What a hardware filter must match to receive frames addressed to `dst`.
#define _Z_CAN_FILTER_ID(dst) (((uint32_t)(dst)&_Z_CAN_NODE_MASK) << _Z_CAN_ID_DST_SHIFT)
#define _Z_CAN_FILTER_MASK (_Z_CAN_NODE_MASK << _Z_CAN_ID_DST_SHIFT)

#define _Z_CAN_PRIO_DEFAULT 5u  // Z_PRIORITY_DATA

// ---------------------------------------------------------------------------
// Segmentation
//
// The link carries a datagram of up to _Z_CAN_MTU_SIZE bytes, which is far more
// than one frame, and segments it itself the way Cyphal/CAN does: the LAST byte
// of every frame is a tail byte, and there is no flow control and no round
// trip. zenoh's own fragmentation therefore never runs on this link; a batch
// up to the MTU is handed to the link whole.
//
//   bit 7    start of transfer
//   bit 6    end of transfer
//   bit 5    toggle: 1 on the first frame, alternating, so a lost frame shows
//   bit 4..0 transfer id, incremented per datagram, so a stale tail is ignored
//
// Because a CAN FD frame length is quantised (8, 12, 16, 20, 24, 32, 48, 64) the
// last frame is padded, and the datagram is therefore prefixed with its length:
// one byte below 128, otherwise two (low seven bits first, bit 7 = continued).
// A transfer that needs more than one frame also ends with a CRC-16/CCITT-FALSE
// over everything before it, padding included, most significant byte first.
#define _Z_CAN_TAIL_SOT 0x80u
#define _Z_CAN_TAIL_EOT 0x40u
#define _Z_CAN_TAIL_TOGGLE 0x20u
#define _Z_CAN_TAIL_TID_MASK 0x1Fu

#define _Z_CAN_FD_FRAME_LEN 64u
#define _Z_CAN_CLASSIC_FRAME_LEN 8u

// Largest datagram. Every peer must agree on it, and on the batch size.
#define _Z_CAN_MTU_SIZE 4095u

// Worst-case transfer on the wire: length prefix, datagram, padding, CRC.
#define _Z_CAN_MAX_TRANSFER (_Z_CAN_MTU_SIZE + 2u + 16u + 2u)

// Concurrent transfers being reassembled. A transfer is keyed by (priority,
// key, source), so a node's urgent message can arrive in the middle of its own
// bulk one. Each slot costs _Z_CAN_MAX_TRANSFER bytes, allocated on first use.
#ifndef Z_CAN_RX_SESSIONS
#define Z_CAN_RX_SESSIONS 4
#endif

typedef struct {
    uint8_t *_buf;
    uint32_t _key;  // identifier with the destination removed
    uint32_t _age;  // for evicting the stalest slot
    uint16_t _len;
    uint8_t _tid;
    uint8_t _toggle;  // toggle bit the next frame must carry
    _Bool _active;
} _z_can_rx_session_t;

typedef struct {
    _z_can_rx_session_t _s[Z_CAN_RX_SESSIONS];
    uint32_t _clock;
} _z_can_rx_t;

typedef struct {
    _z_sys_net_socket_t _sock;
    uint8_t _node;  // this node
    uint8_t _peer;  // unicast: the one node we talk to; else _Z_CAN_NODE_NONE
    uint8_t _prio;  // priority put in the identifier of what we send
    uint8_t _tid;   // next transfer id
    _Bool _fd_mode;
    _z_can_rx_t _rx;
} _z_can_socket_t;

// The sender, reported through the `addr` slice: one byte, the source node.
#define _Z_CAN_ADDR_SIZE 1u

// ---------------------------------------------------------------------------
// Codec: pure, no I/O, so it can be tested without a bus.

/** Emits one frame. `len` is a legal frame length for the mode. */
typedef z_result_t (*_z_can_frame_out_f)(void *ctx, uint32_t id, const uint8_t *data, size_t len);

/** Segment a datagram into frames and hand each to `out`. `frame_max` is 64 for
 *  CAN FD and 8 for classic CAN. Fails if `len` exceeds _Z_CAN_MTU_SIZE. */
z_result_t _z_can_encode(const uint8_t *ptr, size_t len, uint32_t id, uint8_t tid, size_t frame_max,
                         _z_can_frame_out_f out, void *ctx);

/** Feed one received frame. Returns 1 and sets `*out_len` when it completes a
 *  datagram (copied to `out`), 0 when more frames are needed or the frame was
 *  dropped as stale or corrupt. */
int _z_can_decode(_z_can_rx_t *rx, uint32_t id, const uint8_t *data, size_t len, uint8_t *out, size_t out_cap,
                  size_t *out_len);

void _z_can_rx_clear(_z_can_rx_t *rx);

// ---------------------------------------------------------------------------
// Platform: raw frames.

/** Bring the interface up and receive only frames whose destination is
 *  `rx_dst` (the node itself for unicast, _Z_CAN_NODE_BROADCAST for
 *  multicast). `dbitrate` of 0 selects classic CAN; a preconfigured interface
 *  may ignore the rates, but must report the mode it got in `_fd_mode`. */
z_result_t _z_can_dev_open(_z_can_socket_t *sock, const char *dev, uint32_t bitrate, uint32_t dbitrate,
                           uint8_t rx_dst);
void _z_can_dev_close(_z_can_socket_t *sock);
z_result_t _z_can_dev_send(const _z_can_socket_t *sock, uint32_t id, const uint8_t *data, size_t len);
/** Blocks for one frame. `*len` is the capacity on entry (64) and the length on exit. */
z_result_t _z_can_dev_recv(const _z_can_socket_t *sock, uint32_t *id, uint8_t *data, size_t *len);

// ---------------------------------------------------------------------------
// Link

z_result_t _z_open_can(_z_can_socket_t *sock, const char *dev, uint32_t bitrate, uint32_t dbitrate, uint8_t node,
                       uint8_t peer);
void _z_close_can(_z_can_socket_t *sock);

/** Read one datagram. Skips our own frames and, on a unicast link, anyone but
 *  the peer. When `addr` is non-NULL the source node is written to it. Returns
 *  the datagram length, or `SIZE_MAX` on error. */
size_t _z_read_can(_z_can_socket_t *sock, uint8_t *ptr, size_t len, _z_slice_t *addr);

/** Write one datagram of up to _Z_CAN_MTU_SIZE bytes. Returns `len`, or `SIZE_MAX`. */
size_t _z_send_can(_z_can_socket_t *sock, const uint8_t *ptr, size_t len);

#endif

#ifdef __cplusplus
}
#endif

#endif /* ZENOH_PICO_SYSTEM_LINK_CAN_H */
