#include <ESP32DMASPIMaster.h>
#include <ESP32DMASPISlave.h>

#include "helper.h"

ESP32DMASPI::Master master;
ESP32DMASPI::Slave slave;

// Reference: https://rabbit-note.com/2019/01/20/esp32-arduino-spi-slave/

static const uint32_t BUFFER_SIZE = 8192;
uint8_t* spi_master_tx_buf;
uint8_t* spi_master_rx_buf;
uint8_t* spi_slave_tx_buf;
uint8_t* spi_slave_rx_buf;

void setup() {
    Serial.begin(115200);

    // to use DMA buffer, use these methods to allocate buffer
    spi_master_tx_buf = master.allocDMABuffer(BUFFER_SIZE);
    spi_master_rx_buf = master.allocDMABuffer(BUFFER_SIZE);
    spi_slave_tx_buf = slave.allocDMABuffer(BUFFER_SIZE);
    spi_slave_rx_buf = slave.allocDMABuffer(BUFFER_SIZE);

    set_buffer(spi_master_tx_buf, spi_master_rx_buf, spi_slave_tx_buf, spi_slave_rx_buf, BUFFER_SIZE);
    delay(5000);

    // ===== SPI Master =====

    // master device configuration
    master.setDataMode(SPI_MODE0);           // default: SPI_MODE0
    master.setFrequency(4000000);            // default: 8MHz (too fast for bread board...)
    master.setMaxTransferSize(BUFFER_SIZE);  // default: 4092 bytes
    master.setDutyCyclePos(96);              // default: 128 (required for my bread board)

    // begin() after setting
    master.begin(VSPI);  // HSPI (CS: 15, CLK: 14, MOSI: 13, MISO: 12) -> default
                         // VSPI (CS:  5, CLK: 18, MOSI: 23, MISO: 19)

    // ===== SPI Slave =====

    // slave device configuration
    slave.setDataMode(SPI_MODE0);
    slave.setMaxTransferSize(BUFFER_SIZE);

    // begin() after setting
    slave.begin(HSPI);  // HSPI = CS: 15, CLK: 14, MOSI: 13, MISO: 12 -> default
                        // VSPI (CS:  5, CLK: 18, MOSI: 23, MISO: 19)

    // connect same name pins each other
    // CS - CS, CLK - CLK, MOSI - MOSI, MISO - MISO
}

void loop() {
    // just queue transaction
    // if transaction has completed from master, buffer is automatically updated
    if (slave.remained() == 0) {
        slave.queue(spi_slave_rx_buf, spi_slave_tx_buf, BUFFER_SIZE);

        // start and wait to complete transaction
        master.transfer(spi_master_tx_buf, spi_master_rx_buf, BUFFER_SIZE);
    }

    // if slave has received transaction data, available() returns size of received transactions
    while (slave.available()) {
        printf("slave received size = %d\n", slave.size());

        if (memcmp(spi_slave_rx_buf, spi_master_tx_buf, BUFFER_SIZE)) {
            printf("[ERROR] Master -> Slave Received Data has not matched !!\n");
            cmp_bug("Received ", spi_slave_rx_buf, "Sent ", spi_master_tx_buf, BUFFER_SIZE);
        }

        if (memcmp(spi_master_rx_buf, spi_slave_tx_buf, BUFFER_SIZE)) {
            printf("ERROR: Slave -> Master Received Data has not matched !!\n");
            cmp_bug("Received ", spi_master_rx_buf, "Sent ", spi_slave_tx_buf, BUFFER_SIZE);
        }

        slave.pop();

        if (slave.available() == 0) {
            delay(2000);
        }
    }
}
