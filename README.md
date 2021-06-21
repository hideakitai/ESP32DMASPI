# ESP32DMASPI
SPI library for ESP32 which use DMA buffer to send/receive transactions

## Feature

- support DMA buffer (more than 64 byte transfer is available)
- support SPI Slave mode based on [ESP32's SPI Slave Driver](https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/spi_slave.html#spi-slave-driver)
- Master has two ways to send/receive transactions
  - `transfer()` to send/receive transaction one by one
  - `queue()` and `yield()` to send/receive multiple transactions at once (more efficient than `transfer()` many times)
- Slave has two ways to receive/send transactions
  - `wait()` to receive/send transaction one by one
  - `queue()` and `yield()` to receive/send multiple transactions at once (more efficient than `wait()` many times)


## WARNING

- There is known [issue](https://www.esp32.com/viewtopic.php?f=12&t=7339&sid=2257561718efae97d5b805c039b5764e) that last 4 bytes are missing if DMA is used with SPI Slave
  - you need to send 4 bytes more to send all required bytes to ESP32 SPI Slave with DMA
  - see `examples/master_slave_polling_avoid_drop_last_4byte` for more detail
- There is also a known issue that [received data is bit shifted depending on the SPI mode](https://github.com/espressif/esp-idf/search?q=dma+spi+bit+shift&type=issues)
  - Please try SPI mode 0 for this issue
  - But SPI mode 1 or 3 is required based on the [official doc](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/spi_slave.html#restrictions-and-known-issues)
  - Please use this library and SPI modes for your own risk


## Usage

### Master

``` C++
#include <ESP32DMASPIMaster.h>

ESP32DMASPI::Master master;

static const uint32_t BUFFER_SIZE = 8192;
uint8_t* spi_master_tx_buf;
uint8_t* spi_master_rx_buf;

void setup()
{
    Serial.begin(115200);

    // to use DMA buffer, use these methods to allocate buffer
    spi_master_tx_buf = master.allocDMABuffer(BUFFER_SIZE);
    spi_master_rx_buf = master.allocDMABuffer(BUFFER_SIZE);

    master.setDataMode(SPI_MODE3);
    master.setFrequency(SPI_MASTER_FREQ_8M);
    master.setMaxTransferSize(BUFFER_SIZE);
    master.setDMAChannel(1); // 1 or 2 only
    master.setQueueSize(1); // transaction queue size

    // begin() after setting
    // HSPI = CS: 15, CLK: 14, MOSI: 13, MISO: 12
    master.begin(); // default SPI is HSPI
}

void loop()
{
   	// set buffer data here

    // start and wait to complete transaction
    master.transfer(spi_master_tx_buf, spi_master_rx_buf, BUFFER_SIZE);

    // do something here with received data (if needed)
    for (size_t i = 0; i < BUFFER_SIZE; ++i)
        printf("%d ", spi_master_rx_buf[i]);
    printf("\n");

    delay(2000);
}
```



### Slave

``` C++
#include <ESP32DMASPISlave.h>

ESP32DMASPI::Slave slave;

static const uint32_t BUFFER_SIZE = 8192;
uint8_t* spi_slave_tx_buf;
uint8_t* spi_slave_rx_buf;

void setup()
{
    Serial.begin(115200);

    // to use DMA buffer, use these methods to allocate buffer
    spi_slave_tx_buf = slave.allocDMABuffer(BUFFER_SIZE);
    spi_slave_rx_buf = slave.allocDMABuffer(BUFFER_SIZE);

    slave.setDataMode(SPI_MODE3);
    slave.setMaxTransferSize(BUFFER_SIZE);
    slave.setDMAChannel(2); // 1 or 2 only
    slave.setQueueSize(1); // transaction queue size

    // begin() after setting
    // HSPI = CS: 15, CLK: 14, MOSI: 13, MISO: 12
    slave.begin(); // default SPI is HSPI
}

void loop()
{
    // set buffer (reply to master) data here

    // if there is no transaction in queue, add transaction
    if (slave.remained() == 0)
        slave.queue(spi_slave_rx_buf, spi_slave_tx_buf, BUFFER_SIZE);

    // if transaction has completed from master,
    // available() returns size of results of transaction,
    // and buffer is automatically updated

    while (slave.available())
    {
        // do something here with received data
        for (size_t i = 0; i < BUFFER_SIZE; ++i)
            printf("%d ", spi_slave_rx_buf[i]);
        printf("\n");

        slave.pop();
    }
}
```



## TODO (PR welcome!!)

- more configs? (SPI ISR callbacks, SPI cmd/addr/user feature, etc...)


## License

MIT
