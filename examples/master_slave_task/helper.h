#include <Arduino.h>

void dump_buf(const char* title, uint8_t* buf, uint32_t start, uint32_t len) {
    if (len == 1)
        printf("%s [%d]: ", title, start);
    else
        printf("%s [%d-%d]: ", title, start, start + len - 1);

    for (uint32_t i = 0; i < len; i++) {
        printf("%02X ", buf[start + i]);
    }
    printf("\n");
}

void cmp_bug(const char* a_title, uint8_t* a_buf, const char* b_title, uint8_t* b_buf, uint32_t size) {
    for (uint32_t i = 0; i < size; i++) {
        uint32_t j = 1;

        if (a_buf[i] == b_buf[i]) continue;

        while (a_buf[i + j] != b_buf[i + j]) {
            j++;
        }

        dump_buf(a_title, a_buf, i, j);
        dump_buf(b_title, b_buf, i, j);
        i += j - 1;
    }
}

void set_buffer(uint8_t* master_tx, uint8_t* master_rx, uint8_t* slave_tx, uint8_t* slave_rx, const size_t size) {
    for (uint32_t i = 0; i < size; i++) {
        master_tx[i] = i & 0xFF;
        slave_tx[i] = (0xFF - i) & 0xFF;
    }
    memset(master_rx, 0, size);
    memset(slave_rx, 0, size);
}
