/** @file
    BOSS THERM BRTRF Wireless Room Thermostat decoder.

    Copyright (C) 2026 Tom Nicolson

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
 */

#include "decoder.h"

/**
BOSS THERM BRTRF Wireless Room Thermostat (paired with BR1 boiler receiver).

RF and modulation properties (434.026 MHz, OOK Pulse-Distance Modulation):
- Short pulse: ~300-310 us HIGH
- Bit 0: ~300 us HIGH followed by ~300 us LOW
- Bit 1: ~300 us HIGH followed by ~900 us LOW
- Sync / preamble: ~4590 us HIGH followed by ~900 us LOW (sliced as leading '1' bit)
- Trailing stop pulse: ~300 us HIGH
- Inter-frame gap inside a burst: ~22-32 ms (up to ~65 ms across dropped frames)

Frame structure (33 sliced bits = 1 sync bit + 32-bit MSB-first payload):
A button press transmits 8 bursts arranged as 4 alternating pairs of
(normal, bitwise-complement) frames within ~480 ms (`gap_limit = 2000 us`,
`reset_limit = 45000 us` keeps normal/complement pairs in one bitbuffer):

    S PPPPPPPP FBBBBBBB CCCCCCCC DDDDDDDD

- S: 1-bit sync marker from the ~4590 us HIGH + ~900 us LOW preamble (always 1)
- P: Byte 0 (`0x37` on observed unit; unverified across multiple units)
- F: Bit 8 (`0x80` mask in byte 1) toggles between consecutive transmissions;
  remaining bits in bytes 1-2 (`0x0401`) are static on the observed unit
- D: Byte 3 command flag (`0x01` = heating ON, `0x00` = heating OFF)

Integrity & deduplication:
- Every valid command is validated against its exact bitwise-complement
  partner (`inverted == ~normal`) within the same bitbuffer (`mic: "COMPLEMENT"`).
- Isolated frames without a verified complement pair are rejected to prevent
  false positives and inverted-state misinterpretation.
- Repeated pairs within the same burst are deduplicated to emit one event.
- Disabled by default until additional physical thermostat units are captured
  to establish device ID boundaries and multi-unit address ranges.
*/
static int boss_therm_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    uint32_t payloads[BITBUF_ROWS];
    int num_valid = 0;

    if (bitbuffer->num_rows < 2 || bitbuffer->num_rows > BITBUF_ROWS) {
        return DECODE_ABORT_LENGTH;
    }

    for (unsigned r = 0; r < bitbuffer->num_rows; ++r) {
        if (bitbuffer->bits_per_row[r] != 33) {
            continue;
        }
        // Require leading sync bit produced by the ~4590 us HIGH + ~900 us LOW preamble
        if ((bitbuffer->bb[r][0] & 0x80) == 0) {
            continue;
        }

        uint8_t b[4] = {0};
        bitbuffer_extract_bytes(bitbuffer, r, 1, b, 32);
        uint32_t val = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];

        // Reject trivial all-zero or all-one stuck lines
        if (val == 0x00000000u || val == 0xFFFFFFFFu) {
            continue;
        }
        payloads[num_valid++] = val;
    }

    if (num_valid == 0) {
        return DECODE_ABORT_LENGTH;
    }
    if (num_valid < 2) {
        return DECODE_FAIL_MIC;
    }

    // Match normal / bitwise-complement pairs and deduplicate repeated commands
    uint32_t emitted[BITBUF_ROWS];
    int num_emitted      = 0;
    int saw_complement   = 0;
    int saw_valid_cmd    = 0;
    int events           = 0;

    for (int i = 0; i < num_valid; ++i) {
        uint32_t a = payloads[i];

        int has_complement = 0;
        for (int j = 0; j < num_valid; ++j) {
            if (j != i && (a ^ payloads[j]) == 0xFFFFFFFFu) {
                has_complement = 1;
                break;
            }
        }
        if (!has_complement) {
            continue;
        }
        saw_complement = 1;

        // Exactly one of {a, ~a} can have a valid 0x00 (OFF) or 0x01 (ON) command LSB
        uint32_t normal;
        uint8_t cmd_a = a & 0xFFu;
        uint8_t cmd_b = (~a) & 0xFFu;
        if (cmd_a <= 0x01u) {
            normal = a;
        }
        else if (cmd_b <= 0x01u) {
            normal = ~a;
        }
        else {
            continue;
        }
        saw_valid_cmd = 1;

        int already_emitted = 0;
        for (int k = 0; k < num_emitted; ++k) {
            if (emitted[k] == normal) {
                already_emitted = 1;
                break;
            }
        }
        if (already_emitted) {
            continue;
        }
        emitted[num_emitted++] = normal;

        char raw_str[9];
        snprintf(raw_str, sizeof(raw_str), "%08X", normal);
        char const *heating_state = (normal & 0x01u) ? "ON" : "OFF";

        /* clang-format off */
        data_t *data = data_make(
                "model",    "",             DATA_STRING, "BOSS-THERM-BRTRF",
                "heating",  "Heating",      DATA_STRING, heating_state,
                "raw",      "Raw Code",     DATA_STRING, raw_str,
                "mic",      "Integrity",    DATA_STRING, "COMPLEMENT",
                NULL);
        /* clang-format on */
        decoder_output_data(decoder, data);
        events++;
    }

    if (events > 0) {
        return events;
    }
    if (!saw_complement) {
        return DECODE_FAIL_MIC;
    }
    if (!saw_valid_cmd) {
        return DECODE_FAIL_SANITY;
    }
    return 0;
}

static char const *const output_fields[] = {
        "model",
        "heating",
        "raw",
        "mic",
        NULL,
};

r_device const boss_therm = {
        .name        = "BOSS THERM BRTRF Wireless Room Thermostat",
        .modulation  = OOK_PULSE_PPM,
        .short_width = 300,
        .long_width  = 900,
        .gap_limit   = 2000,
        .reset_limit = 45000,
        .tolerance   = 120,
        .decode_fn   = &boss_therm_decode,
        .disabled    = 1, // single-unit protocol capture; complement-pair check only
        .fields      = output_fields,
};
