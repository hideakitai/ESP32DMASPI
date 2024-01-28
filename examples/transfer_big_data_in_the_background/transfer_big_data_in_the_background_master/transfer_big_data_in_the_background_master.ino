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
    master.begin();  // default: HSPI (CS: 15, CLK: 14, MOSI: 13, MISO: 12)

    delay(2000);

    Serial.println("start spi master");
}

void loop()
{
    // if no transaction is in flight and interval has passed, queue new transactions
    static uint32_t last_sent_ms = millis();
    if (master.numTransactionsInFlight() == 0 && millis() > last_sent_ms + 2000) {
        // initialize tx/rx buffers
        Serial.println("initialize tx/rx buffers");
        initializeBuffers(dma_tx_buf, dma_rx_buf, BUFFER_SIZE, 0);

        // queue transaction and trigger it right now
        Serial.println("execute transaction in the background");
        master.queue(dma_tx_buf, dma_rx_buf, BUFFER_SIZE);
        master.trigger();

        Serial.println("next transaction will be sent 2 seconds after");
        last_sent_ms = millis();
    }

    // you can do some other stuff here
    // NOTE: you can't touch dma_tx/rx_buf because it's in-flight in the background

    // if you need, you can handle transfer result (you can ignore this if you don't need)
    while (master.numTransactionsCompleted()) {
        // get the oldest transfer result
        const size_t received_bytes = master.numBytesReceived();

        // verify and dump difference with received data
        if (verifyAndDumpDifference("master", dma_tx_buf, BUFFER_SIZE, "slave", dma_rx_buf, received_bytes)) {
            Serial.println("successfully received expected data from slave");
        } else {
            Serial.println("Unexpected difference found between master/slave data");
        }
    }
}
