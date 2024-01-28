#include <ESP32DMASPIMaster.h>
#include "helper.h"

ESP32DMASPI::Master master;

static constexpr size_t BUFFER_SIZE = 256;
static constexpr size_t QUEUE_SIZE = 1;
uint8_t *dma_tx_buf;
uint8_t *dma_rx_buf;

void setup()
{
    Serial.begin(115200);

    delay(2000);

    // to use DMA buffer, use these methods to allocate buffer
    dma_tx_buf = master.allocDMABuffer(BUFFER_SIZE);
    dma_rx_buf = master.allocDMABuffer(BUFFER_SIZE);

    master.setDataMode(SPI_MODE0);           // default: SPI_MODE0
    master.setFrequency(1000000);            // default: 8MHz (too fast for bread board...)
    master.setMaxTransferSize(BUFFER_SIZE);  // default: 4092 bytes
    master.setQueueSize(QUEUE_SIZE);         // default: 1

    // begin() after setting
    master.begin();  // default: HSPI (please refer README for pin assignments)

    delay(2000);

    Serial.println("start spi master");
}

void loop()
{
    // initialize tx/rx buffers
    initializeBuffers(dma_tx_buf, dma_rx_buf, BUFFER_SIZE);

    // start and wait to complete one BIG transaction (same data will be received from slave)
    const size_t received_bytes = master.transfer(dma_tx_buf, dma_rx_buf, BUFFER_SIZE);

    // verify and dump difference with received data
    if (verifyAndDumpDifference("master", dma_tx_buf, BUFFER_SIZE, "slave", dma_rx_buf, received_bytes)) {
        Serial.println("successfully received expected data from slave");
    } else {
        Serial.println("unexpected difference found between master/slave data");
    }

    delay(2000);
}
