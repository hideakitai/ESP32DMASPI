#include <ESP32DMASPIMaster.h>
#include <ESP32DMASPISlave.h>

ESP32DMASPI::Master master;
ESP32DMASPI::Slave slave;

// Reference: https://rabbit-note.com/2019/01/20/esp32-arduino-spi-slave/

static const uint32_t BUFFER_SIZE = 1024;
static const uint8_t N_QUEUES = 4;
uint8_t* spi_master_tx_buf[N_QUEUES];
uint8_t* spi_master_rx_buf[N_QUEUES];
uint8_t* spi_slave_tx_buf[N_QUEUES];
uint8_t* spi_slave_rx_buf[N_QUEUES];

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

void set_buffer() {
    for (uint8_t q = 0; q < N_QUEUES; ++q) {
        for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
            spi_master_tx_buf[q][i] = i & 0xFF;
            spi_slave_tx_buf[q][i] = (0xFF - i) & 0xFF;
        }
        memset(spi_master_rx_buf[q], 0, BUFFER_SIZE);
        memset(spi_slave_rx_buf[q], 0, BUFFER_SIZE);
    }
}

void setup() {
    Serial.begin(115200);

    // to use DMA buffer, use these methods to allocate buffer
    for (uint8_t i = 0; i < N_QUEUES; ++i) {
        spi_master_tx_buf[i] = master.allocDMABuffer(BUFFER_SIZE);
        spi_master_rx_buf[i] = master.allocDMABuffer(BUFFER_SIZE);
        spi_slave_tx_buf[i] = slave.allocDMABuffer(BUFFER_SIZE);
        spi_slave_rx_buf[i] = slave.allocDMABuffer(BUFFER_SIZE);
    }

    set_buffer();

    delay(5000);

    master.setDataMode(SPI_MODE3);
    // master.setFrequency(SPI_MASTER_FREQ_8M); // too fast for bread board...
    master.setFrequency(4000000);
    master.setMaxTransferSize(BUFFER_SIZE);
    master.setDMAChannel(1);        // 1 or 2 only
    master.setQueueSize(N_QUEUES);  // transaction queue size
    // begin() after setting
    // VSPI = CS: 5, CLK: 18, MOSI: 23, MISO: 19
    master.begin(VSPI);

    slave.setDataMode(SPI_MODE3);
    slave.setMaxTransferSize(BUFFER_SIZE);
    slave.setDMAChannel(2);        // 1 or 2 only
    slave.setQueueSize(N_QUEUES);  // transaction queue size
    // begin() after setting
    // HSPI = CS: 15, CLK: 14, MOSI: 13, MISO: 12
    slave.begin(HSPI);

    // connect same name pins each other
    // CS - CS, CLK - CLK, MOSI - MOSI, MISO - MISO
}

void loop() {
    // queue transaction
    for (uint8_t i = 0; i < N_QUEUES; ++i) {
        slave.queue(spi_slave_rx_buf[i], spi_slave_tx_buf[i], BUFFER_SIZE);
    }

    static uint32_t count = 0;
    if (count++ % 3 == 0) {
        // queue transaction
        for (uint8_t i = 0; i < N_QUEUES; ++i) {
            master.queue(spi_master_tx_buf[i], spi_master_rx_buf[i], BUFFER_SIZE);
        }

        // wait until transaction will complete
        master.yield();
    }

    // if slave has received transaction data, available() returns size of received transactions
    while (slave.available() < N_QUEUES)
        ;

    for (size_t q = 0; q < N_QUEUES; ++q) {
        printf("slave received size = %d\n", slave.size());

        if (memcmp(spi_slave_rx_buf[q], spi_master_tx_buf[q], BUFFER_SIZE)) {
            printf("[ERROR] Master -> Slave Received Data has not matched !!\n");
            cmp_bug("Received ", spi_slave_rx_buf[q], "Sent ", spi_master_tx_buf[q], BUFFER_SIZE);
        }

        if (memcmp(spi_master_rx_buf[q], spi_slave_tx_buf[q], BUFFER_SIZE)) {
            printf("ERROR: Slave -> Master Received Data has not matched !!\n");
            cmp_bug("Received ", spi_master_rx_buf[q], "Sent ", spi_slave_tx_buf[q], BUFFER_SIZE);
        }

        slave.pop();
    }

    static uint32_t prev_ms = millis();
    printf("wait for next loop.. elapsed = %ld\n", millis() - prev_ms);
    prev_ms = millis();
    delay(2000);
}
