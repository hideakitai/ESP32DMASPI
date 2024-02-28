#include <ESP32DMASPIMaster.h>
#include "helper.h"

ESP32DMASPI::Master master;

static constexpr size_t BUFFER_SIZE = 256; // should be multiple of 4
static constexpr size_t QUEUE_SIZE = 1;
uint8_t *dma_tx_buf;
uint8_t *dma_rx_buf;

// user-defined callback arguments
volatile static size_t count_pre_cb = 0;
volatile static size_t count_post_cb = 0;
// definition of user callback
void IRAM_ATTR userTransactionCallback(spi_transaction_t *trans, void *arg)
{
    // NOTE: here is an ISR Context
    //       there are significant limitations on what can be done with ISRs,
    //       so use this feature carefully!

    // convert user-defined argument from (void *) -> (size_t *)
    size_t *count = (size_t *)arg;
    // increment count (this should be locked or atomic operation, but this is simplified)
    *count += 1;
}

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
    // print counts if changed
    // (this should be locked or atomic operation, but this is simplified)
    static size_t prev_count_pre_cb = count_pre_cb;
    if (count_pre_cb != prev_count_pre_cb) {
        Serial.printf("count_pre_cb changed: %u -> %u\n", prev_count_pre_cb, count_pre_cb);
        prev_count_pre_cb = count_pre_cb;
    }
    static size_t prev_count_post_cb = count_post_cb;
    if (count_post_cb != prev_count_post_cb) {
        Serial.printf("count_post_cb changed: %u -> %u\n", prev_count_post_cb, count_post_cb);
        prev_count_post_cb = count_post_cb;
    }

    // if no transaction is in flight and interval has passed, queue new transactions
    static uint32_t last_sent_ms = millis();
    if (master.numTransactionsInFlight() == 0 && millis() > last_sent_ms + 2000) {
        // initialize tx/rx buffers
        Serial.println("initialize tx/rx buffers");
        initializeBuffers(dma_tx_buf, dma_rx_buf, BUFFER_SIZE, 0);

        Serial.println("execute transaction in the background with callbacks");
        // with user-defined ISR callback that is called before/after transaction start
        // you can set these callbacks and arguments before each queue()
        master.setUserPreCbAndArg(userTransactionCallback, (void *)&count_pre_cb);
        master.setUserPostCbAndArg(userTransactionCallback, (void *)&count_post_cb);
        // queue transaction and trigger it right now
        master.queue(dma_tx_buf, dma_rx_buf, BUFFER_SIZE);
        master.trigger();

        Serial.println("next transaction will be sent 2 seconds after");
        last_sent_ms = millis();
    }

    // you can do some other stuff here
    // NOTE: you can't touch dma_tx/rx_buf because it's in-flight in the background

    // if you need, you can handle transfer result (you can ignore this if you don't need)
    while (master.numTransactionsCompleted()) {
        // process received data from slave
        Serial.println("all queued transactions completed. start verifying received data from slave");

        // get the oldest transfer result
        const size_t received_bytes = master.numBytesReceived();

        // verify and dump difference with received data
        if (verifyAndDumpDifference("master", dma_tx_buf, BUFFER_SIZE, "slave", dma_rx_buf, received_bytes)) {
            Serial.println("successfully received expected data from slave");
        } else {
            Serial.println("unexpected difference found between master/slave data");
        }
    }
}
