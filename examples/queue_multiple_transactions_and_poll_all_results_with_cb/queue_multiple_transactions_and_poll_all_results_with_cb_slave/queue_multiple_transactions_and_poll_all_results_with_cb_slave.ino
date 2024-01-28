#include <ESP32DMASPISlave.h>
#include "helper.h"

ESP32DMASPI::Slave slave;

static constexpr size_t BUFFER_SIZE = 256;
static constexpr size_t QUEUE_SIZE = 2;
uint8_t *dma_tx_buf;
uint8_t *dma_rx_buf;

// definition of user callback
static constexpr int LEVEL_HIGH = HIGH;
static constexpr int LEVEL_LOW = LOW;
void userTransactionCallback(spi_slave_transaction_t *trans, void *arg)
{
    // NOTE: here is ISR context
    int level = *((int *)arg);
#ifdef CONFIG_IDF_TARGET_ESP32
    digitalWrite(2, level);
#else
    digitalWrite(LED_BUILTIN, level);
#endif
}

void setup()
{
    Serial.begin(115200);

    delay(2000);

    // to use DMA buffer, use these methods to allocate buffer
    dma_tx_buf = slave.allocDMABuffer(BUFFER_SIZE);
    dma_rx_buf = slave.allocDMABuffer(BUFFER_SIZE);

    slave.setDataMode(SPI_MODE0);           // default: SPI_MODE0
    slave.setMaxTransferSize(BUFFER_SIZE);  // default: 4092 bytes
    slave.setQueueSize(QUEUE_SIZE);         // default: 1, requres 2 in this example

    // begin() after setting
    slave.begin();  // default: HSPI (please refer README for pin assignments)

    Serial.println("start spi slave");
}

void loop()
{
    // if no transaction is in flight and all results are handled, queue new transactions
    if (slave.numTransactionsInFlight() == 0 && slave.numTransactionsCompleted() == 0) {
        // initialize tx/rx buffers
        Serial.println("initialize tx/rx buffers");
        initializeBuffers(dma_tx_buf, dma_rx_buf, BUFFER_SIZE, 0);

        // queue multiple transactions
        Serial.println("queue multiple transactions");
        // in this example, the master sends some data first
        // with user-defined ISR callback that is called before/after transaction start
        slave.setUserPostSetupCbAndArg(userTransactionCallback, (void *)&LEVEL_HIGH);
        slave.setUserPostTransCbAndArg(userTransactionCallback, (void *)&LEVEL_LOW);
        slave.queue(NULL, dma_rx_buf, BUFFER_SIZE);
        // and the slave sends same data after that
        // with user-defined ISR callback that is called before/after transaction start
        slave.setUserPostSetupCbAndArg(userTransactionCallback, (void *)&LEVEL_HIGH);
        slave.setUserPostTransCbAndArg(userTransactionCallback, (void *)&LEVEL_LOW);
        slave.queue(dma_tx_buf, NULL, BUFFER_SIZE);

        // finally, we should trigger transaction in the background
        slave.trigger();

        Serial.println("wait for the completion of the queued transactions...");
    }

    // you can do some other stuff here
    // NOTE: you can't touch dma_tx/rx_buf because it's in-flight in the background

    // if all transactions are completed and all results are ready, handle results
    if (slave.numTransactionsInFlight() == 0 && slave.numTransactionsCompleted() == QUEUE_SIZE) {
        // process received data from slave
        Serial.println("all queued transactions completed. start verifying received data from slave");

        // get received bytes for all transactions
        const std::vector<size_t> received_bytes = slave.numBytesReceivedAll();

        // verify and dump difference with received data
        // NOTE: we need only 1st results (received_bytes[0])
        if (verifyAndDumpDifference("slave", dma_tx_buf, BUFFER_SIZE, "master", dma_rx_buf, received_bytes[0])) {
            Serial.println("successfully received expected data from master");
        } else {
            Serial.println("Unexpected difference found between master/slave data");
        }
    }
}
