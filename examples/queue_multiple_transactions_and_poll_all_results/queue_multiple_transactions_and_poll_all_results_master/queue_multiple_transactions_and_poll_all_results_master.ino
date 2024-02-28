#include <ESP32DMASPIMaster.h>
#include "helper.h"

ESP32DMASPI::Master master;

static constexpr size_t BUFFER_SIZE = 256; // should be multiple of 4
static constexpr size_t QUEUE_SIZE = 2;
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
    master.setQueueSize(QUEUE_SIZE);         // default: 1, requres 2 in this example

    // begin() after setting
    master.begin();  // default: HSPI (CS: 15, CLK: 14, MOSI: 13, MISO: 12)

    delay(2000);

    Serial.println("start spi master");
}

void loop()
{
    // if no transaction is in flight and all results are handled, queue new transactions
    if (master.hasTransactionsCompletedAndAllResultsHandled()) {
        // initialize tx/rx buffers
        Serial.println("initialize tx/rx buffers");
        initializeBuffers(dma_tx_buf, dma_rx_buf, BUFFER_SIZE, 0);

        // queue multiple transactions
        Serial.println("queue multiple transactions");
        // in this example, the master sends some data first,
        master.queue(dma_tx_buf, NULL, BUFFER_SIZE);
        // and the slave sends same data after that
        master.queue(NULL, dma_rx_buf, BUFFER_SIZE);

        // finally, we should trigger transaction in the background
        master.trigger();

        Serial.println("wait for the completion of the queued transactions...");
    }

    // you can do some other stuff here
    // NOTE: you can't touch dma_tx/rx_buf because it's in-flight in the background

    // if all transactions are completed and all results are ready, handle results
    if (master.hasTransactionsCompletedAndAllResultsReady(QUEUE_SIZE)) {
        // process received data from slave
        Serial.println("all queued transactions completed. start verifying received data from slave");

        // get received bytes for all transactions
        const std::vector<size_t> received_bytes = master.numBytesReceivedAll();

        // verify and dump difference with received data
        // NOTE: we need only 2nd results (received_bytes[1])
        if (verifyAndDumpDifference("master", dma_tx_buf, BUFFER_SIZE, "slave", dma_rx_buf, received_bytes[1])) {
            Serial.println("successfully received expected data from slave");
        } else {
            Serial.println("unexpected difference found between master/slave data");
        }

        Serial.println("wait for 2 seconds for next transaction...");
        delay(2000);
    }
}
