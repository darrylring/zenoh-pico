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

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "zenoh-pico/config.h"
#include "zenoh-pico/link/transport/can.h"
#include "zenoh-pico/system/platform.h"
#include "zenoh-pico/utils/logging.h"

#if Z_FEATURE_LINK_CAN == 1

static uint16_t __z_can_crc16(uint16_t crc, uint8_t b) {
    crc ^= (uint16_t)((uint16_t)b << 8);
    for (int i = 0; i < 8; i++) {
        crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}

// Smallest legal frame length that holds `n` bytes.
static size_t __z_can_frame_len(size_t n, size_t frame_max) {
    if (n <= 8u) {
        return n;
    }
    static const uint8_t steps[] = {12, 16, 20, 24, 32, 48, 64};
    for (size_t i = 0; i < sizeof(steps); i++) {
        if ((n <= steps[i]) && (steps[i] <= frame_max)) {
            return steps[i];
        }
    }
    return frame_max;
}

// Length prefix: 1 byte below 128, else 2 (low 7 bits first, bit 7 = more).
static size_t __z_can_put_len(uint8_t *dst, size_t len) {
    if (len < 0x80u) {
        dst[0] = (uint8_t)len;
        return 1;
    }
    dst[0] = (uint8_t)(0x80u | (len & 0x7Fu));
    dst[1] = (uint8_t)(len >> 7);
    return 2;
}

// Returns bytes consumed, or 0 if malformed.
static size_t __z_can_get_len(const uint8_t *src, size_t avail, size_t *len) {
    if (avail < 1u) {
        return 0;
    }
    if ((src[0] & 0x80u) == 0u) {
        *len = src[0];
        return 1;
    }
    if ((avail < 2u) || (src[1] >= 0x80u)) {
        return 0;
    }
    *len = (size_t)(src[0] & 0x7Fu) | ((size_t)src[1] << 7);
    return 2;
}

z_result_t _z_can_encode(const uint8_t *ptr, size_t len, uint32_t id, uint8_t tid, size_t frame_max,
                         _z_can_frame_out_f out, void *ctx) {
    if (len > _Z_CAN_MTU_SIZE) {
        _Z_ERROR("CAN: datagram %zu exceeds link MTU %u", len, (unsigned)_Z_CAN_MTU_SIZE);
        return _Z_ERR_GENERIC;
    }

    uint8_t prefix[2];
    size_t plen = __z_can_put_len(prefix, len);
    size_t total = plen + len;
    size_t cap = frame_max - 1u;
    tid &= _Z_CAN_TAIL_TID_MASK;

    // The stream is prefix | datagram | padding | crc, and the crc exists only
    // when it spans several frames. Padding is zeros, sized so that the last
    // frame is a legal length.
    size_t pad;
    size_t crc_len;
    if (total < frame_max) {
        crc_len = 0;
        pad = __z_can_frame_len(total + 1u, frame_max) - (total + 1u);
    } else {
        crc_len = 2;
        size_t l0 = total + crc_len;
        size_t last = l0 - (((l0 + cap - 1u) / cap) - 1u) * cap;
        pad = __z_can_frame_len(last + 1u, frame_max) - (last + 1u);
    }
    size_t stream = total + pad + crc_len;
    size_t body = total + pad;

    uint16_t crc = 0xFFFFu;
    if (crc_len != 0u) {
        for (size_t i = 0; i < body; i++) {
            uint8_t b = (i < plen) ? prefix[i] : (i < total) ? ptr[i - plen] : 0u;
            crc = __z_can_crc16(crc, b);
        }
    }

    uint8_t frame[_Z_CAN_FD_FRAME_LEN];
    uint8_t toggle = _Z_CAN_TAIL_TOGGLE;
    for (size_t pos = 0; pos < stream;) {
        size_t n = ((stream - pos) < cap) ? (stream - pos) : cap;
        for (size_t i = 0; i < n; i++) {
            size_t p = pos + i;
            uint8_t b;
            if (p < plen) {
                b = prefix[p];
            } else if (p < total) {
                b = ptr[p - plen];
            } else if (p < body) {
                b = 0;
            } else {
                b = (p == body) ? (uint8_t)(crc >> 8) : (uint8_t)(crc & 0xFFu);
            }
            frame[i] = b;
        }
        uint8_t tail = (uint8_t)(toggle | tid);
        if (pos == 0u) {
            tail |= _Z_CAN_TAIL_SOT;
        }
        if ((pos + n) == stream) {
            tail |= _Z_CAN_TAIL_EOT;
        }
        frame[n] = tail;

        z_result_t ret = out(ctx, id, frame, n + 1u);
        if (ret != _Z_RES_OK) {
            return ret;
        }
        pos += n;
        toggle ^= _Z_CAN_TAIL_TOGGLE;
    }
    return _Z_RES_OK;
}

// Take the datagram out of a finished stream.
static int __z_can_deliver(const uint8_t *stream, size_t n, uint8_t *out, size_t out_cap, size_t *out_len) {
    size_t dlen = 0;
    size_t plen = __z_can_get_len(stream, n, &dlen);
    if ((plen == 0u) || (dlen > (n - plen)) || (dlen > out_cap)) {
        return 0;
    }
    memcpy(out, stream + plen, dlen);
    *out_len = dlen;
    return 1;
}

static _z_can_rx_session_t *__z_can_session(_z_can_rx_t *rx, uint32_t key, _Bool claim) {
    _z_can_rx_session_t *free_slot = NULL;
    _z_can_rx_session_t *oldest = NULL;
    for (size_t i = 0; i < Z_CAN_RX_SESSIONS; i++) {
        _z_can_rx_session_t *s = &rx->_s[i];
        if (s->_active && (s->_key == key)) {
            return s;
        }
        if (!s->_active) {
            free_slot = s;
        } else if ((oldest == NULL) || ((rx->_clock - s->_age) > (rx->_clock - oldest->_age))) {
            oldest = s;
        }
    }
    if (!claim) {
        return NULL;
    }
    return (free_slot != NULL) ? free_slot : oldest;
}

int _z_can_decode(_z_can_rx_t *rx, uint32_t id, const uint8_t *data, size_t len, uint8_t *out, size_t out_cap,
                  size_t *out_len) {
    if (len < 1u) {
        return 0;
    }
    uint8_t tail = data[len - 1u];
    size_t n = len - 1u;
    _Bool sot = (tail & _Z_CAN_TAIL_SOT) != 0u;
    _Bool eot = (tail & _Z_CAN_TAIL_EOT) != 0u;
    uint8_t toggle = tail & _Z_CAN_TAIL_TOGGLE;
    uint8_t tid = tail & _Z_CAN_TAIL_TID_MASK;
    uint32_t key = id & ~(_Z_CAN_FILTER_MASK);

    if (sot && eot) {
        // Single frame. A transfer in flight under the same key is abandoned.
        _z_can_rx_session_t *s = __z_can_session(rx, key, false);
        if (s != NULL) {
            s->_active = false;
        }
        return (toggle != 0u) ? __z_can_deliver(data, n, out, out_cap, out_len) : 0;
    }

    _z_can_rx_session_t *s;
    if (sot) {
        if (toggle == 0u) {
            return 0;
        }
        s = __z_can_session(rx, key, true);
        if (s->_buf == NULL) {
            s->_buf = (uint8_t *)z_malloc(_Z_CAN_MAX_TRANSFER);
            if (s->_buf == NULL) {
                return 0;
            }
        }
        s->_active = true;
        s->_key = key;
        s->_tid = tid;
        s->_len = 0;
        s->_toggle = _Z_CAN_TAIL_TOGGLE;
    } else {
        s = __z_can_session(rx, key, false);
        if ((s == NULL) || (s->_tid != tid) || (s->_toggle != toggle)) {
            // A frame went missing, or this is the tail of something we never
            // saw the start of. Drop the transfer; the sender's next one starts clean.
            if (s != NULL) {
                s->_active = false;
            }
            return 0;
        }
    }

    if (((size_t)s->_len + n) > _Z_CAN_MAX_TRANSFER) {
        s->_active = false;
        return 0;
    }
    memcpy(s->_buf + s->_len, data, n);
    s->_len = (uint16_t)(s->_len + n);
    s->_toggle ^= _Z_CAN_TAIL_TOGGLE;
    s->_age = ++rx->_clock;

    if (!eot) {
        return 0;
    }
    s->_active = false;
    if (s->_len < 3u) {
        return 0;
    }
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < s->_len; i++) {
        crc = __z_can_crc16(crc, s->_buf[i]);
    }
    if (crc != 0u) {
        _Z_DEBUG("CAN: transfer from node %u failed its CRC", (unsigned)_Z_CAN_ID_SRC(id));
        return 0;
    }
    return __z_can_deliver(s->_buf, (size_t)s->_len - 2u, out, out_cap, out_len);
}

void _z_can_rx_clear(_z_can_rx_t *rx) {
    for (size_t i = 0; i < Z_CAN_RX_SESSIONS; i++) {
        if (rx->_s[i]._buf != NULL) {
            z_free(rx->_s[i]._buf);
        }
    }
    memset(rx, 0, sizeof(*rx));
}

#endif
