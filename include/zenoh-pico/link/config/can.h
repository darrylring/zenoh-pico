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

#ifndef ZENOH_PICO_LINK_CONFIG_CAN_H
#define ZENOH_PICO_LINK_CONFIG_CAN_H

#include "zenoh-pico/collections/intmap.h"
#include "zenoh-pico/collections/string.h"
#include "zenoh-pico/config.h"
#include "zenoh-pico/system/platform.h"
#include "zenoh-pico/utils/result.h"

#ifdef __cplusplus
extern "C" {
#endif

#if Z_FEATURE_LINK_CAN == 1

// Endpoint: can/<device>#bitrate=<n>;dbitrate=<n>;id=<n>;peer=<n>
//
//   device   the CAN interface name ("can0", "vcan0", or on Zephyr the
//            devicetree node label)
//   bitrate  arbitration-phase bit rate; also the sole rate for classic CAN
//   dbitrate CAN FD data-phase bit rate. 0 selects classic CAN (8-byte frames)
//   id       THIS node, 0..253. It is the source in every identifier we send.
//            Left out on a link that connects, the node asks the router for one
//            (see link/transport/can.h)
//   peer     0..253. Makes the link UNICAST to that one node.
//
// One endpoint syntax, one frame format, two link kinds, the way `udp` does it:
// a locator that is listened on is multicast and every frame goes to all nodes
// (destination 0xFF); a locator that is connected to is unicast and needs
// `peer`, and every frame goes to that node and hears only that node.
//
// Node ids must be unique on the bus. CAN does not allow two nodes to send
// different data under one identifier, and the source is part of it. Asking for
// one is how a device with nothing but a long unique id gets one.
//
// The identifier is the bus priority as well as the address: lowest wins
// arbitration, and the zenoh priority sits in its high bits, so an urgent
// message beats a bulk one on the wire regardless of which node sent it. See
// link/transport/can.h for the layout.

#define CAN_CONFIG_ARGC 4

#define CAN_CONFIG_BITRATE_KEY 0x01
#define CAN_CONFIG_BITRATE_STR "bitrate"

#define CAN_CONFIG_DBITRATE_KEY 0x02
#define CAN_CONFIG_DBITRATE_STR "dbitrate"

#define CAN_CONFIG_ID_KEY 0x03
#define CAN_CONFIG_ID_STR "id"

#define CAN_CONFIG_PEER_KEY 0x04
#define CAN_CONFIG_PEER_STR "peer"

#define CAN_CONFIG_MAPPING_BUILD                    \
    _z_str_intmapping_t args[CAN_CONFIG_ARGC];      \
    args[0]._key = CAN_CONFIG_BITRATE_KEY;          \
    args[0]._str = (char *)CAN_CONFIG_BITRATE_STR;  \
    args[1]._key = CAN_CONFIG_DBITRATE_KEY;         \
    args[1]._str = (char *)CAN_CONFIG_DBITRATE_STR; \
    args[2]._key = CAN_CONFIG_ID_KEY;               \
    args[2]._str = (char *)CAN_CONFIG_ID_STR;       \
    args[3]._key = CAN_CONFIG_PEER_KEY;             \
    args[3]._str = (char *)CAN_CONFIG_PEER_STR;

// Defaults when a key is absent from the endpoint.
#define CAN_CONFIG_DEFAULT_BITRATE 500000u
#define CAN_CONFIG_DEFAULT_DBITRATE 2000000u
#define CAN_CONFIG_DEFAULT_ID 1u

size_t _z_can_config_strlen(const _z_str_intmap_t *s);
void _z_can_config_onto_str(char *dst, size_t dst_len, const _z_str_intmap_t *s);
char *_z_can_config_to_str(const _z_str_intmap_t *s);
z_result_t _z_can_config_from_str(_z_str_intmap_t *strint, const char *s);
z_result_t _z_can_config_from_strn(_z_str_intmap_t *strint, const char *s, size_t n);

#endif

#ifdef __cplusplus
}
#endif

#endif /* ZENOH_PICO_LINK_CONFIG_CAN_H */
