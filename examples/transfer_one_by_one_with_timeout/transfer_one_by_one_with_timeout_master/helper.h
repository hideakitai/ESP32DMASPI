#pragma once

#include <Arduino.h>
#include <cstdint>
#include <cstddef>

void dumpBuffers(const char *title, const uint8_t *buf, size_t start, size_t len)
{
    // show title and range
    if (len == 1)
        printf("%s [%d]: ", title, start);
    else
        printf("%s [%d-%d]: ", title, start, start + len - 1);

    // show data in the range
    for (size_t i = 0; i < len; i++) {
        printf("%02X ", buf[start + i]);
    }
    printf("\n");
}

bool verifyAndDumpDifference(const char *a_title, const uint8_t *a_buf, size_t a_size, const char *b_title, const uint8_t *b_buf, size_t b_size)
{
    bool verified = true;

    if (a_size != b_size) {
        printf("received data size does not match: expected = %d / actual = %d\n", a_size, b_size);
        return false;
    }

    for (size_t i = 0; i < a_size; i++) {
        // if a_buf and b_buf is same, continue
        if (a_buf[i] == b_buf[i]) {
            continue;
        }

        verified = false;

        // if a_buf[i] and b_buf[i] is not same, check the range that has difference
        size_t j = 1;
        for (; i + j < a_size; ++j) {
            if (a_buf[i + j] != b_buf[i + j]) {
                break;
            }
        }

        // dump different data range
        dumpBuffers(a_title, a_buf, i, j);
        dumpBuffers(b_title, b_buf, i, j);

        // restart from next same index (-1 considers i++ in for())
        i += j - 1;
    }
    return verified;
}

void initializeBuffers(uint8_t *tx, uint8_t *rx, size_t size, size_t offset = 0)
{
    if (tx) {
        for (size_t i = 0; i < size; i++) {
            tx[i] = (i + offset) & 0xFF;
        }
    }
    if (rx) {
        memset(rx, 0, size);
    }
}
