#include <ESP32DMASPIMaster.h>
#include <ESP32DMASPISlave.h>

#include "helper.h"

ESP32DMASPI::Master master;
ESP32DMASPI::Slave slave;

// Reference: https://rabbit-note.com/2019/01/20/esp32-arduino-spi-slave/

static const uint32_t BUFFER_SIZE = 1024;
static const uint8_t N_QUEUES = 4;
uint8_t* spi_master_tx_buf[N_QUEUES];
uint8_t* spi_master_rx_buf[N_QUEUES];
uint8_t* spi_slave_tx_buf[N_QUEUES];
uint8_t* spi_slave_rx_buf[N_QUEUES];

void setup() {
    Serial.begin(115200);

    // to use DMA buffer, use these methods to allocate buffer
    for (uint8_t i = 0; i < N_QUEUES; ++i) {
        spi_master_tx_buf[i] = master.allocDMABuffer(BUFFER_SIZE);
        spi_master_rx_buf[i] = master.allocDMABuffer(BUFFER_SIZE);
        spi_slave_tx_buf[i] = slave.allocDMABuffer(BUFFER_SIZE);
        spi_slave_rx_buf[i] = slave.allocDMABuffer(BUFFER_SIZE);
    }

    set_buffer(spi_master_tx_buf, spi_master_rx_buf, spi_slave_tx_buf, spi_slave_rx_buf, N_QUEUES, BUFFER_SIZE);
    delay(5000);

    // ===== SPI Master =====

    // master device configuration
    master.setDataMode(SPI_MODE0);           // default: SPI_MODE0
    master.setFrequency(4000000);            // default: 8MHz (too fast for bread board...)
    master.setMaxTransferSize(BUFFER_SIZE);  // default: 4092 bytes
    master.setDutyCyclePos(96);              // default: 128 (required for my bread board)
    master.setQueueSize(N_QUEUES);           // transaction queue size

    // begin() after setting
    master.begin(VSPI);  // HSPI (CS: 15, CLK: 14, MOSI: 13, MISO: 12) -> default
                         // VSPI (CS:  5, CLK: 18, MOSI: 23, MISO: 19)

    // ===== SPI Slave =====

    // slave device configuration
    slave.setDataMode(SPI_MODE0);
    slave.setMaxTransferSize(BUFFER_SIZE);
    slave.setQueueSize(N_QUEUES);  // transaction queue size

    // begin() after setting
    slave.begin(HSPI);  // HSPI = CS: 15, CLK: 14, MOSI: 13, MISO: 12 -> default
                        // VSPI (CS:  5, CLK: 18, MOSI: 23, MISO: 19)

    // connect same name pins each other
    // CS - CS, CLK - CLK, MOSI - MOSI, MISO - MISO
}

void loop() {
    // queue transaction
    for (uint8_t i = 0; i < N_QUEUES; ++i) {
        slave.queue(spi_slave_rx_buf[i], spi_slave_tx_buf[i], BUFFER_SIZE);
    }

    // queue transaction
    for (uint8_t i = 0; i < N_QUEUES; ++i) {
        master.queue(spi_master_tx_buf[i], spi_master_rx_buf[i], BUFFER_SIZE);
    }

    // wait until transaction will complete
    master.yield();

    // if slave has received transaction data, available() returns size of received transactions
    while (slave.available() < N_QUEUES)
        ;

    const size_t received_transactions = slave.available();
    for (size_t q = 0; q < received_transactions; ++q) {
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
    printf("wait for next loop.. elapsed = %ld\n", millis() - prev_ms - 2000);
    prev_ms = millis();
    delay(2000);
}
