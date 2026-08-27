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

// Test for the CAN link: the segmentation codec (no bus needed) and, when a CAN
// interface exists, the whole link over it. Create a virtual interface with:
//     sudo modprobe vcan
//     sudo ip link add dev vcan0 type vcan
//     sudo ip link set vcan0 mtu 72    (CAN FD frames need this; without it the link uses classic CAN)
//     sudo ip link set up vcan0
// The bus part skips when the interface is absent, so it is safe in CI.

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "zenoh-pico/config.h"

#if Z_FEATURE_LINK_CAN == 1

#include <net/if.h>

#include "zenoh-pico/link/transport/can.h"

#define CAN_DEV (getenv("ZP_CAN_DEV") != NULL ? getenv("ZP_CAN_DEV") : "vcan0")

static int failures = 0;

static void check(const char *what, int ok) {
    printf("  %-60s %s\n", what, ok ? "ok" : "FAILED");
    if (!ok) {
        failures++;
    }
}

// ---- codec ----------------------------------------------------------------

typedef struct {
    uint8_t data[200][64];
    size_t len[200];
    uint32_t id[200];
    size_t n;
} frames_t;

static z_result_t collect(void *ctx, uint32_t id, const uint8_t *data, size_t len) {
    frames_t *f = (frames_t *)ctx;
    f->id[f->n] = id;
    f->len[f->n] = len;
    memcpy(f->data[f->n], data, len);
    f->n++;
    return _Z_RES_OK;
}

static int legal_len(size_t n, size_t frame_max) {
    if (n <= 8) return 1;
    if (frame_max < 64) return 0;
    return n == 12 || n == 16 || n == 20 || n == 24 || n == 32 || n == 48 || n == 64;
}

static uint8_t big_in[_Z_CAN_MTU_SIZE];
static uint8_t big_out[_Z_CAN_MTU_SIZE];
static frames_t frames;

static void codec_roundtrip(size_t len, size_t frame_max) {
    char label[80];
    snprintf(label, sizeof(label), "%s: %zu bytes", frame_max == 64 ? "FD" : "classic", len);
    for (size_t i = 0; i < len; i++) big_in[i] = (uint8_t)(i * 7u + 1u);

    memset(&frames, 0, sizeof(frames));
    _z_can_rx_t rx;
    memset(&rx, 0, sizeof(rx));
    uint32_t id = _Z_CAN_ID_MAKE(3, 0, 0xFF, 0x12);
    if (_z_can_encode(big_in, len, id, 9, frame_max, collect, &frames) != _Z_RES_OK) {
        check(label, 0);
        return;
    }
    int ok = 1;
    size_t got = 0;
    int done = 0;
    for (size_t i = 0; i < frames.n; i++) {
        ok &= legal_len(frames.len[i], frame_max);
        uint8_t tail = frames.data[i][frames.len[i] - 1];
        ok &= ((tail & 0x80) != 0) == (i == 0);
        ok &= ((tail & 0x40) != 0) == (i == frames.n - 1);
        ok &= ((tail & 0x20) != 0) == (i % 2 == 0);
        ok &= (tail & 0x1F) == 9;
        int r = _z_can_decode(&rx, frames.id[i], frames.data[i], frames.len[i], big_out, sizeof(big_out), &got);
        ok &= (r == 1) == (i == frames.n - 1);
        done |= r;
    }
    ok &= done && got == len && memcmp(big_in, big_out, len) == 0;
    check(label, ok);
    _z_can_rx_clear(&rx);
}

static void codec_tests(void) {
    printf("codec\n");
    // Around every DLC step, the single/multi-frame boundary, the 1/2-byte
    // length prefix and the MTU.
    static const size_t lens[] = {0,  1,  6,  7,  8,  10, 11, 14, 15,  18,  22,   30,   46,  60,
                                  61, 62, 63, 64, 65, 100, 127, 128, 200, 1000, 4000, 4094, 4095};
    for (size_t i = 0; i < sizeof(lens) / sizeof(lens[0]); i++) {
        codec_roundtrip(lens[i], 64);
        if (lens[i] <= 1000) codec_roundtrip(lens[i], 8);
    }
    // Every length at both frame sizes.
    int ok = 1;
    for (size_t len = 0; len <= 300; len++) {
        memset(&frames, 0, sizeof(frames));
        _z_can_rx_t rx;
        memset(&rx, 0, sizeof(rx));
        size_t got = 0, fm = (len & 1) ? 8 : 64;
        for (size_t i = 0; i < len; i++) big_in[i] = (uint8_t)(len + i);
        if (_z_can_encode(big_in, len, 1, 0, fm, collect, &frames) != _Z_RES_OK) { ok = 0; continue; }
        int r = 0;
        for (size_t i = 0; i < frames.n; i++)
            r = _z_can_decode(&rx, frames.id[i], frames.data[i], frames.len[i], big_out, sizeof(big_out), &got);
        ok &= r == 1 && got == len && memcmp(big_in, big_out, len) == 0;
        _z_can_rx_clear(&rx);
    }
    check("every length 0..300 round-trips", ok);

    {  // over MTU is refused
        memset(&frames, 0, sizeof(frames));
        static uint8_t over[_Z_CAN_MTU_SIZE + 1];
        check("over-MTU datagram is refused", _z_can_encode(over, sizeof(over), 1, 0, 64, collect, &frames) != _Z_RES_OK);
    }

    {  // a lost frame drops the transfer, and the next one is clean
        for (size_t i = 0; i < 300; i++) big_in[i] = (uint8_t)i;
        memset(&frames, 0, sizeof(frames));
        _z_can_rx_t rx;
        memset(&rx, 0, sizeof(rx));
        size_t got = 0;
        _z_can_encode(big_in, 300, 1, 0, 64, collect, &frames);
        int any = 0;
        for (size_t i = 0; i < frames.n; i++) {
            if (i == 2) continue;
            any |= _z_can_decode(&rx, frames.id[i], frames.data[i], frames.len[i], big_out, sizeof(big_out), &got);
        }
        check("transfer with a lost frame is dropped", any == 0);
        frames.n = 0;
        _z_can_encode(big_in, 300, 1, 1, 64, collect, &frames);
        int r = 0;
        for (size_t i = 0; i < frames.n; i++)
            r = _z_can_decode(&rx, frames.id[i], frames.data[i], frames.len[i], big_out, sizeof(big_out), &got);
        check("next transfer after the loss is delivered", r == 1 && got == 300 && memcmp(big_in, big_out, 300) == 0);
        _z_can_rx_clear(&rx);
    }

    {  // a corrupted byte fails the CRC
        memset(&frames, 0, sizeof(frames));
        _z_can_rx_t rx;
        memset(&rx, 0, sizeof(rx));
        size_t got = 0;
        _z_can_encode(big_in, 300, 1, 0, 64, collect, &frames);
        frames.data[1][5] ^= 0x40;
        int r = 0;
        for (size_t i = 0; i < frames.n; i++)
            r = _z_can_decode(&rx, frames.id[i], frames.data[i], frames.len[i], big_out, sizeof(big_out), &got);
        check("corrupted transfer fails its CRC", r == 0);
        _z_can_rx_clear(&rx);
    }

    {  // two sources interleaved frame by frame
        static frames_t fa, fb;
        memset(&fa, 0, sizeof(fa));
        memset(&fb, 0, sizeof(fb));
        static uint8_t da[400], db[400], oa[400], ob[400];
        for (size_t i = 0; i < 400; i++) { da[i] = (uint8_t)i; db[i] = (uint8_t)(255 - i); }
        _z_can_encode(da, 400, _Z_CAN_ID_MAKE(5, 0, 0xFF, 1), 0, 64, collect, &fa);
        _z_can_encode(db, 400, _Z_CAN_ID_MAKE(1, 0, 0xFF, 2), 0, 64, collect, &fb);
        _z_can_rx_t rx;
        memset(&rx, 0, sizeof(rx));
        size_t ga = 0, gb = 0;
        int ra = 0, rb = 0;
        for (size_t i = 0; i < fa.n; i++) {
            ra |= _z_can_decode(&rx, fa.id[i], fa.data[i], fa.len[i], oa, sizeof(oa), &ga);
            rb |= _z_can_decode(&rx, fb.id[i], fb.data[i], fb.len[i], ob, sizeof(ob), &gb);
        }
        check("interleaved transfers from two nodes both arrive",
              ra && rb && ga == 400 && gb == 400 && !memcmp(da, oa, 400) && !memcmp(db, ob, 400));
        _z_can_rx_clear(&rx);
    }

    {  // identifier packing
        uint32_t id = _Z_CAN_ID_MAKE(5, 17, 0x42, 0x07);
        check("identifier fields round-trip", _Z_CAN_ID_PRIO(id) == 5 && _Z_CAN_ID_KEY(id) == 17 &&
                                                  _Z_CAN_ID_DST(id) == 0x42 && _Z_CAN_ID_SRC(id) == 7);
        check("lower priority value wins arbitration",
              _Z_CAN_ID_MAKE(1, 0, 0xFF, 200) < _Z_CAN_ID_MAKE(5, 0, 0xFF, 1));
    }
}

// ---- bus ------------------------------------------------------------------

static void bus_roundtrip(_z_can_socket_t *a, _z_can_socket_t *b, size_t len, const char *label) {
    static uint8_t out[_Z_CAN_MTU_SIZE], in[_Z_CAN_MTU_SIZE];
    for (size_t i = 0; i < len; i++) out[i] = (uint8_t)(i * 7u + 1u);
    if (_z_send_can(a, out, len) != len) {
        check(label, 0);
        return;
    }
    memset(in, 0, sizeof(in));
    uint8_t addrbuf[32] = {0};
    _z_slice_t addr = _z_slice_alias_buf(addrbuf, sizeof(addrbuf));
    size_t rb = _z_read_can(b, in, sizeof(in), &addr);
    check(label, rb == len && memcmp(in, out, len) == 0 && addr.len == _Z_CAN_ADDR_SIZE && addrbuf[0] == a->_node);
}

static void bus_tests(const char *dev) {
    printf("bus '%s'\n", dev);
    _z_can_socket_t a, b, c;
    // a and b: multicast peers. c and b2 below: a unicast pair.
    if (_z_open_can(&a, dev, 500000u, 2000000u, 1, _Z_CAN_NODE_NONE) != _Z_RES_OK ||
        _z_open_can(&b, dev, 500000u, 2000000u, 2, _Z_CAN_NODE_NONE) != _Z_RES_OK) {
        check("open multicast peers", 0);
        return;
    }
    printf("  mode: %s\n", a._fd_mode ? "CAN FD" : "classic CAN");
    bus_roundtrip(&a, &b, 0, "multicast: empty");
    bus_roundtrip(&a, &b, 40, "multicast: 40 bytes (one frame)");
    bus_roundtrip(&a, &b, 99, "multicast: 99 bytes (the Join with QoS)");
    bus_roundtrip(&b, &a, 1500, "multicast: 1500 bytes, reverse");
    bus_roundtrip(&a, &b, 4095, "multicast: 4095 bytes (MTU)");
    check("over-MTU write is refused", _z_send_can(&a, big_in, _Z_CAN_MTU_SIZE + 1u) == SIZE_MAX);

    // Unicast pair 10 <-> 11, with a bystander 12 that must hear nothing.
    _z_can_socket_t u1, u2;
    if (_z_open_can(&u1, dev, 500000u, 2000000u, 10, 11) != _Z_RES_OK ||
        _z_open_can(&u2, dev, 500000u, 2000000u, 11, 10) != _Z_RES_OK ||
        _z_open_can(&c, dev, 500000u, 2000000u, 12, 10) != _Z_RES_OK) {
        check("open unicast pair", 0);
        return;
    }
    bus_roundtrip(&u1, &u2, 700, "unicast: 700 bytes, 10 -> 11");
    bus_roundtrip(&u2, &u1, 50, "unicast: 50 bytes, 11 -> 10");

    // Bystander c (unicast to 10, node 12) must not see 10 -> 11, and the
    // multicast peers must not see unicast traffic.
    _z_send_can(&u1, big_in, 20);
    _z_send_can(&a, big_in, 21);
    uint8_t buf[64];
    size_t r = _z_read_can(&u2, buf, sizeof(buf), NULL);
    check("unicast peer gets its datagram", r == 20);
    r = _z_read_can(&b, buf, sizeof(buf), NULL);
    check("multicast peer skips unicast traffic", r == 21);

    _z_close_can(&a);
    _z_close_can(&b);
    _z_close_can(&c);
    _z_close_can(&u1);
    _z_close_can(&u2);
}

int main(void) {
    printf("z_can_link_test\n");
    codec_tests();
    if (if_nametoindex(CAN_DEV) == 0) {
        printf("no interface '%s' -- skipping the bus tests\n", CAN_DEV);
    } else {
        bus_tests(CAN_DEV);
    }
    printf("%s (%d failure%s)\n", (failures == 0) ? "PASS" : "FAIL", failures, (failures == 1) ? "" : "s");
    return (failures == 0) ? 0 : 1;
}

#else

int main(void) {
    printf("z_can_link_test: built without Z_FEATURE_LINK_CAN -- skipping\n");
    return 0;
}

#endif
