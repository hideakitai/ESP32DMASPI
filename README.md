# ESP32DMASPI

SPI library for ESP32 which use DMA buffer to send/receive transactions

## ESP32SPISlave

This is the SPI library to send/receive large transaction with DMA. Please use [ESP32SPISlave](https://github.com/hideakitai/ESP32SPISlave) for the simple SPI Slave mode without DMA.

## Feature

- support DMA buffer (more than 64 byte transfer is available)
- support both SPI Master and Slave mode based on ESP32's [SPI Master Driver](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html#) and [SPI Slave Driver](https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/spi_slave.html#spi-slave-driver)
- Master has two ways to send/receive transactions
  - `transfer()` to send/receive transaction one by one
  - `queue()` and `yield()` to send/receive multiple transactions at once (more efficient than `transfer()` many times)
- Slave has two ways to receive/send transactions
  - `wait()` to receive/send transaction one by one
  - `queue()` and `yield()` to receive/send multiple transactions at once (more efficient than `wait()` many times)
- Various configurations based on driver APIs

## WARNING

- There is known [issue](https://www.esp32.com/viewtopic.php?f=12&t=7339&sid=2257561718efae97d5b805c039b5764e) that last 4 bytes are missing if DMA is used with SPI Slave
  - you need to send 4 bytes more to send all required bytes to ESP32 SPI Slave with DMA
- There is also a known issue that [received data is bit shifted depending on the SPI mode](https://github.com/espressif/esp-idf/search?q=dma+spi+bit+shift&type=issues)
  - Please try SPI mode 0 for this issue
  - But SPI mode 1 or 3 is required based on the [official doc](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/spi_slave.html#restrictions-and-known-issues)
  - Please use this library and SPI modes for your own risk

## Usage

### Master

```C++
#include <ESP32DMASPIMaster.h>

ESP32DMASPI::Master master;

static const uint32_t BUFFER_SIZE = 8192;
uint8_t* spi_master_tx_buf;
uint8_t* spi_master_rx_buf;

void setup() {
    // to use DMA buffer, use these methods to allocate buffer
    spi_master_tx_buf = master.allocDMABuffer(BUFFER_SIZE);
    spi_master_rx_buf = master.allocDMABuffer(BUFFER_SIZE);

    // set buffer data...

    master.setDataMode(SPI_MODE0);           // default: SPI_MODE0
    master.setFrequency(4000000);            // default: 8MHz (too fast for bread board...)
    master.setMaxTransferSize(BUFFER_SIZE);  // default: 4092 bytes

    // begin() after setting
    master.begin();  // HSPI (CS: 15, CLK: 14, MOSI: 13, MISO: 12) -> default
                     // VSPI (CS:  5, CLK: 18, MOSI: 23, MISO: 19)
}

void loop() {
    // start and wait to complete transaction
    master.transfer(spi_master_tx_buf, spi_master_rx_buf, BUFFER_SIZE);

    // do something with received data if you want
}
```

### Slave

```C++
#include <ESP32DMASPISlave.h>

ESP32DMASPI::Slave slave;

static const uint32_t BUFFER_SIZE = 8192;
uint8_t* spi_slave_tx_buf;
uint8_t* spi_slave_rx_buf;

void setup() {
    // to use DMA buffer, use these methods to allocate buffer
    spi_slave_tx_buf = slave.allocDMABuffer(BUFFER_SIZE);
    spi_slave_rx_buf = slave.allocDMABuffer(BUFFER_SIZE);

    // set buffer data...

    // slave device configuration
    slave.setDataMode(SPI_MODE0);
    slave.setMaxTransferSize(BUFFER_SIZE);

    // begin() after setting
    slave.begin();  // HSPI = CS: 15, CLK: 14, MOSI: 13, MISO: 12 -> default
                    // VSPI (CS:  5, CLK: 18, MOSI: 23, MISO: 19)
}

void loop() {
    // if there is no transaction in queue, add transaction
    if (slave.remained() == 0) {
        slave.queue(spi_slave_rx_buf, spi_slave_tx_buf, BUFFER_SIZE);
    }

    // if transaction has completed from master,
    // available() returns size of results of transaction,
    // and buffer is automatically updated

    while (slave.available()) {
        // do something with received data: spi_slave_rx_buf

        slave.pop();
    }
}
```

## APIs

### Master

```C++
bool begin(const uint8_t spi_bus = HSPI);
bool begin(const uint8_t spi_bus, const int8_t sck, const int8_t miso, const int8_t mosi, const int8_t ss);
bool end();

uint8_t* allocDMABuffer(const size_t n);

// execute transaction and wait for transmission one by one
size_t transfer(const uint8_t* tx_buf, const size_t size);
size_t transfer(const uint8_t* tx_buf, uint8_t* rx_buf, const size_t size);
size_t transfer(const uint16_t cmd, const uint64_t addr, const uint8_t* tx_buf, const size_t size);
size_t transfer(const uint16_t cmd, const uint64_t addr, const uint8_t* tx_buf, uint8_t* rx_buf, const size_t size);
size_t transfer(const uint8_t command_bits, const uint8_t address_bits, const uint16_t cmd, const uint64_t addr, const uint8_t* tx_buf, uint8_t* rx_buf, const size_t size);
size_t transfer(const uint8_t command_bits, const uint8_t address_bits, const uint8_t dummy_bits, const uint32_t flags, const uint16_t cmd, const uint64_t addr, const uint8_t* tx_buf, uint8_t* rx_buf, const size_t size);

// queueing transaction and execute simultaneously
// wait (blocking) and timeout occurs if queue is full with transaction
// (but designed not to queue transaction more than queue_size, so there is no timeout argument)
bool queue(const uint8_t* tx_buf, const size_t size);
bool queue(const uint8_t* tx_buf, uint8_t* rx_buf, const size_t size);
bool queue(const uint16_t cmd, const uint64_t addr, const uint8_t* tx_buf, const size_t size);
bool queue(const uint16_t cmd, const uint64_t addr, const uint8_t* tx_buf, uint8_t* rx_buf, const size_t size);
bool queue(const uint8_t command_bits, const uint8_t address_bits, const uint16_t cmd, const uint64_t addr, const uint8_t* tx_buf, uint8_t* rx_buf, const size_t size);
bool queue(const uint8_t command_bits, const uint8_t address_bits, const uint8_t dummy_bits, const uint32_t flags, const uint16_t cmd, const uint64_t addr, const uint8_t* tx_buf, uint8_t* rx_buf, const size_t size);
void yield();

// ===== Main Configurations =====
// set these optional parameters before begin() if you want
void setDataMode(const uint8_t m);  // SPI_MODE0, 1, 2, 3
void setFrequency(const size_t f);  // up to 80 MHz
void setMaxTransferSize(const size_t n);

// ===== Optional Configurations =====
// set these optional parameters before begin() if you want
void setCommandBits(const uint8_t n);           // 0-16
void setAddressBits(const uint8_t n);           // 0-64
void setDummyBits(const uint8_t n);
void setDutyCyclePos(const uint8_t n);          // 128 for 50%
void setSpiMode(const uint8_t m);               // SPI_MODE0, 1, 2, 3
void setCsEnaPostTrans(const uint8_t n);
void setClockSpeedHz(const size_t f);           // up to 80 MHz
void setInputDelayNs(const int n);
void setDeviceFlags(const uint32_t flags);      // OR of SPI_DEVICE_* flags
void setQueueSize(const size_t n);              // default: 3
void setPreCb(const transaction_cb_t pre_cb);
void setPostCb(const transaction_cb_t post_cb);
void setDMAChannel(const uint8_t c);            // default: auto
```

### Slave

```C++
bool begin(const uint8_t spi_bus = HSPI);
bool begin(const uint8_t spi_bus, const int8_t sck, const int8_t miso, const int8_t mosi, const int8_t ss);
bool end();

uint8_t* allocDMABuffer(const size_t s);

// wait for transaction one by one
bool wait(uint8_t* rx_buf, const size_t size);  // no data to master
bool wait(uint8_t* rx_buf, const uint8_t* tx_buf, const size_t size);

// queueing transaction
// wait (blocking) and timeout occurs if queue is full with transaction
// (but designed not to queue transaction more than queue_size, so there is no timeout argument)
bool queue(uint8_t* rx_buf, const size_t size);  // no data to master
bool queue(uint8_t* rx_buf, const uint8_t* tx_buf, const size_t size);

// wait until all queued transaction will be done by master
// if yield is finished, all the buffer is updated to latest
void yield();

// transaction result info
size_t available() const;
size_t remained() const;
uint32_t size() const;
void pop();

// ===== Main Configurations =====
// set these optional parameters before begin() if you want
void setDataMode(const uint8_t m);
void setMaxTransferSize(const int n);

// ===== Optional Configurations =====
void setSlaveFlags(const uint32_t flags);  // OR of SPI_SLAVE_* flags
void setQueueSize(const int n);
void setSpiMode(const uint8_t m);
void setDMAChannel(const uint8_t c);  // default: auto
```

## Configuration Flags

```C++
#define SPI_DEVICE_TXBIT_LSBFIRST  (1<<0)  ///< Transmit command/address/data LSB first instead of the default MSB first
#define SPI_DEVICE_RXBIT_LSBFIRST  (1<<1)  ///< Receive data LSB first instead of the default MSB first
#define SPI_DEVICE_BIT_LSBFIRST    (SPI_DEVICE_TXBIT_LSBFIRST|SPI_DEVICE_RXBIT_LSBFIRST) ///< Transmit and receive LSB first
#define SPI_DEVICE_3WIRE           (1<<2)  ///< Use MOSI (=spid) for both sending and receiving data
#define SPI_DEVICE_POSITIVE_CS     (1<<3)  ///< Make CS positive during a transaction instead of negative
#define SPI_DEVICE_HALFDUPLEX      (1<<4)  ///< Transmit data before receiving it, instead of simultaneously
#define SPI_DEVICE_CLK_AS_CS       (1<<5)  ///< Output clock on CS line if CS is active
/** There are timing issue when reading at high frequency (the frequency is related to whether iomux pins are used, valid time after slave sees the clock).
  *     - In half-duplex mode, the driver automatically inserts dummy bits before reading phase to fix the timing issue. Set this flag to disable this feature.
  *     - In full-duplex mode, however, the hardware cannot use dummy bits, so there is no way to prevent data being read from getting corrupted.
  *       Set this flag to confirm that you're going to work with output only, or read without dummy bits at your own risk.
  */
#define SPI_DEVICE_NO_DUMMY        (1<<6)
#define SPI_DEVICE_DDRCLK          (1<<7)
```

https://github.com/espressif/esp-idf/blob/master/components/driver/include/driver/spi_master.h#L32-L45

```C++
#define SPI_SLAVE_TXBIT_LSBFIRST          (1<<0)  ///< Transmit command/address/data LSB first instead of the default MSB first
#define SPI_SLAVE_RXBIT_LSBFIRST          (1<<1)  ///< Receive data LSB first instead of the default MSB first
#define SPI_SLAVE_BIT_LSBFIRST            (SPI_SLAVE_TXBIT_LSBFIRST|SPI_SLAVE_RXBIT_LSBFIRST) ///< Transmit and receive LSB first
```

https://github.com/espressif/esp-idf/blob/733fbd9ecc8ac0780de51b3761a16d1faec63644/components/driver/include/driver/spi_slave.h#L23-L25

## Restrictions and Known Issues for SPI with DMA Buffer (Help Wanted)

As mentioned above, in short, there are two restrictions/issues for SPI with DMA buffer.

1. RX buffer and host transfer length should be aligned to 4 bytes (length must be multiples of 4 bytes)
2. Correct transfer requires SPI mode 1 and 3; otherwise Slave -> Master data corrupt (maybe MSB of the first byte)

To avoid 2nd issue, I set `setDutyCyclePos(96)` in examples but it's a workaround and not a good solution. In addition, SPI Mode 1 and 3 do not work correctly with the current configuration if `esp32-arduino` version > `1.0.4`. Also, in the [example of esp-idf](https://github.com/espressif/esp-idf/blob/733fbd9ecc8ac0780de51b3761a16d1faec63644/examples/peripherals/spi_slave/receiver/main/app_main.c), SPI mode 1 is used... I hope someone sends PR for a better solution (configuration) someday!

Please refer [here](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_slave.html#restrictions-and-known-issues) for more information and configure your slave by yourself.

## TODO (PR welcome!!)

- [Dual/Quad SPI](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html#transaction-line-mode)
- [Polling Transaction](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html#polling-transactions)

## Reference

- [ESP-IDF Programming Guide / API Reference / SPI Master Driver](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html#)
- [ESP-IDF Programming Guide / API Reference / SPI Slave Driver](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_slave.html#)

## License

MIT
