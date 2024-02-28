#include <ESP32DMASPISlave.h>
#include "helper.h"

ESP32DMASPI::Slave slave;

static constexpr size_t BUFFER_SIZE = 256; // should be multiple of 4
static constexpr size_t QUEUE_SIZE = 1;
uint8_t *dma_tx_buf;
uint8_t *dma_rx_buf;

void setup()
{
    Serial.begin(115200);

    delay(2000);

    // to use DMA buffer, use these methods to allocate buffer
    dma_tx_buf = slave.allocDMABuffer(BUFFER_SIZE);
    dma_rx_buf = slave.allocDMABuffer(BUFFER_SIZE);

    slave.setDataMode(SPI_MODE0);           // default: SPI_MODE0
    slave.setMaxTransferSize(BUFFER_SIZE);  // default: 4092 bytes
    slave.setQueueSize(QUEUE_SIZE);         // default: 1

    // begin() after setting
    slave.begin();  // default: HSPI (please refer README for pin assignments)

    Serial.println("start spi slave");
}

void loop()
{
    // initialize tx/rx buffers
    initializeBuffers(dma_tx_buf, dma_rx_buf, BUFFER_SIZE);

    // start and wait to complete one BIG transaction (same data will be received from slave)
    const size_t received_bytes = slave.transfer(dma_tx_buf, dma_rx_buf, BUFFER_SIZE, 2000);

    // verify and dump difference with received data
    if (verifyAndDumpDifference("slave", dma_tx_buf, BUFFER_SIZE, "master", dma_rx_buf, received_bytes)) {
        Serial.println("successfully received expected data from master");
    } else {
        Serial.println("unexpected difference found between master/slave data");
    }
}
