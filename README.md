# ESP32DMASPI

SPI library for ESP32 which use DMA buffer to send/receive transactions

> [!CAUTION]
> Breaking API changes from v0.4.0 and above

## ESP32SPISlave

This is the SPI library to send/receive large transactions with DMA. Please use [ESP32SPISlave](https://github.com/hideakitai/ESP32SPISlave) for the simple SPI Slave mode without DMA.

## Feature

- Supports DMA buffer (more than 64 byte transfer is available)
- Supports both SPI Master and Slave mode based on ESP32's [SPI Master Driver](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html#) and [SPI Slave Driver](https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/spi_slave.html#spi-slave-driver)
- Master and Slave has several ways to send/receive transactions
  - `transfer()` to send/receive transaction one by one (blocking)
  - `queue()` and `wait()` to send/receive multiple transactions at once and wait for them (blocking but more efficient than `transfer()` many times)
  - `queue()` and `trigger()` to send/receive multiple transactions at once in the background (non-blocking)
- Various configurations based on driver APIs
- Register user-defined ISR callbacks

### Supported ESP32 Version

| IDE         | ESP32 Board Version |
| ----------- | ------------------- |
| Arduino IDE | `>= 2.0.11`         |
| PlatformIO  | `>= 5.0.0`          |

## Known Issues for ESP32 SPI driver

- There is known [issue](https://www.esp32.com/viewtopic.php?f=12&t=7339&sid=2257561718efae97d5b805c039b5764e) that last 4 bytes are missing if DMA is used with SPI Slave
  - you need to send 4 bytes more to send all required bytes to ESP32 SPI Slave with DMA
- There is also a known issue that [received data is bit shifted depending on the SPI mode](https://github.com/espressif/esp-idf/search?q=dma+spi+bit+shift&type=issues)
  - Please try SPI mode 0 for this issue
  - But SPI mode 1 or 3 is required based on the [official doc](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/spi_slave.html#restrictions-and-known-issues)
  - Please use this library and SPI modes for your own risk

## Notes for Communication Errors

If you have communication errors when trying examples, please check the following points.

- Check that the SPI Mode is the same
- Check if the pin number and connection destination are correct
- Connect pins as short as possible
- Be careful of signal line crosstalk (Be careful not to tangle wires)
- If you are using two devices, ensure they share a common ground level
- If you still have communication problems, try a lower frequency (1MHz or less)

## Usage

Please refer examples for more information.

### Blocking big `transfer()` one by one

#### Master

```C++
#include <ESP32DMASPIMaster.h>

ESP32DMASPI::Master master;

static constexpr size_t BUFFER_SIZE = 256;
static constexpr size_t QUEUE_SIZE = 1;
uint8_t *dma_tx_buf;
uint8_t *dma_rx_buf;

void setup()
{
    // to use DMA buffer, use these methods to allocate buffer
    dma_tx_buf = master.allocDMABuffer(BUFFER_SIZE);
    dma_rx_buf = master.allocDMABuffer(BUFFER_SIZE);

    master.setDataMode(SPI_MODE0);           // default: SPI_MODE0
    master.setFrequency(1000000);            // default: 8MHz
    master.setMaxTransferSize(BUFFER_SIZE);  // default: 4092 bytes
    master.setQueueSize(QUEUE_SIZE);         // default: 1

    // begin() after setting
    master.begin();  // default: HSPI (please refer README for pin assignments)
}

void loop()
{
    // do some initialization for tx_buf and rx_buf

    // start and wait to complete one BIG transaction (same data will be received from slave)
    const size_t received_bytes = master.transfer(dma_tx_buf, dma_rx_buf, BUFFER_SIZE);

    // do something with received_bytes and rx_buf if needed
}
```

#### Slave

```C++
#include <ESP32DMASPISlave.h>

ESP32DMASPI::Slave slave;

static constexpr size_t BUFFER_SIZE = 256;
static constexpr size_t QUEUE_SIZE = 1;
uint8_t *dma_tx_buf;
uint8_t *dma_rx_buf;

void setup()
{
    // to use DMA buffer, use these methods to allocate buffer
    dma_tx_buf = slave.allocDMABuffer(BUFFER_SIZE);
    dma_rx_buf = slave.allocDMABuffer(BUFFER_SIZE);

    slave.setDataMode(SPI_MODE0);           // default: SPI_MODE0
    slave.setMaxTransferSize(BUFFER_SIZE);  // default: 4092 bytes
    slave.setQueueSize(QUEUE_SIZE);         // default: 1

    // begin() after setting
    slave.begin();  // default: HSPI (please refer README for pin assignments)
}

void loop()
{
    // do some initialization for tx_buf and rx_buf

    // start and wait to complete one BIG transaction (same data will be received from slave)
    const size_t received_bytes = slave.transfer(dma_tx_buf, dma_rx_buf, BUFFER_SIZE);

    // do something with received_bytes and rx_buf if needed
}
```

### Blocking multiple transactions

You can use Master and Slave almost the same way (omit the Slave example here).

```c++
void loop()
{
    // do some initialization for tx_buf and rx_buf

    // queue multiple transactions
    // in this example, the master sends some data first,
    master.queue(dma_tx_buf, NULL, BUFFER_SIZE);
    // and the slave sends same data after that
    master.queue(NULL, dma_rx_buf, BUFFER_SIZE);

    // wait for the completion of the queued transactions
    const std::vector<size_t> received_bytes = master.wait();

    // do something with received_bytes and rx_buf if needed
}

```

### Non-blocking multiple transactions

You can use Master and Slave almost the same way (omit the Slave example here).

```c++
void loop()
{
    // if no transaction is in flight and all results are handled, queue new transactions
    if (master.hasTransactionsCompletedAndAllResultsHandled()) {
        // do some initialization for tx_buf and rx_buf

        // queue multiple transactions
        // in this example, the master sends some data first,
        master.queue(dma_tx_buf, NULL, BUFFER_SIZE);
        // and the slave sends same data after that
        master.queue(NULL, dma_rx_buf, BUFFER_SIZE);

        // finally, we should trigger transaction in the background
        master.trigger();
    }

    // you can do some other stuff here
    // NOTE: you can't touch dma_tx/rx_buf because it's in-flight in the background

    // if all transactions are completed and all results are ready, handle results
    if (master.hasTransactionsCompletedAndAllResultsReady(QUEUE_SIZE)) {
        // get received bytes for all transactions
        const std::vector<size_t> received_bytes = master.numBytesReceivedAll();

        // do something with received_bytes and rx_buf if needed
    }
}
```

## SPI Buses and SPI Pins

This library's `bool begin(const uint8_t spi_bus = HSPI)` function uses `HSPI` as the default SPI bus as same as `SPI` library of `arduino-esp32` ([reference](https://github.com/espressif/arduino-esp32/blob/099b432d10fb4ca1529c52241bcadcb8a4386f17/libraries/SPI/src/SPI.h#L61)).

The pins for SPI buses are automatically attached as follows. "Default SPI Pins" means the pins defined there are the same as `MOSI`, `MISO`, `SCK`, and `SS`.

| Board     | SPI       | MOSI | MISO | SCK | SS  | Default SPI Pins |
| --------- | --------- | ---- | ---- | --- | --- | ---------------- |
| `esp32`   | HSPI      | 13   | 12   | 14  | 15  | No               |
| `esp32`   | VSPI/FSPI | 23   | 19   | 18  | 5   | Yes              |
| `esp32s2` | HSPI/FSPI | 35   | 37   | 36  | 34  | Yes              |
| `esp32s3` | HSPI/FSPI | 11   | 13   | 12  | 10  | Yes              |
| `esp32c3` | HSPI/FSPI | 6    | 5    | 4   | 7   | Yes              |

Depending on your board, the default SPI pins are defined in `pins_arduino.h`. For example, `esp32`'s default SPI pins are found [here](https://github.com/espressif/arduino-esp32/blob/e1f14331f173a00a9062f616bc9a62c358b9076f/variants/esp32/pins_arduino.h#L20-L23) (`MOSI: 23, MISO: 19, SCK: 18, SS: 5`). Please refer to [arduino-esp32/variants](https://github.com/espressif/arduino-esp32/tree/e1f14331f173a00a9062f616bc9a62c358b9076f/variants) for your board's default SPI pins.

The supported SPI buses are different from the ESP32 chip. Please note that there may be a restriction to use `FSPI` for your SPI bus. (Note: though `arduino-esp32` still uses `FSPI` and `HSPI` for all chips (v2.0.11), these are deprecated for the chips after `esp32s2`)

| Chip     | FSPI                | HSPI                | VSPI                 |
| -------- | ------------------- | ------------------- | -------------------- |
| ESP32    | SPI1_HOST(`0`) [^1] | SPI2_HOST(`1`) [^2] | SPI3_HOST (`2`) [^3] |
| ESP32-S2 | SPI2_HOST(`1`)      | SPI3_HOST(`2`)      | -                    |
| ESP32-S3 | SPI2_HOST(`1`)      | SPI3_HOST(`2`)      | -                    |
| ESP32-C3 | SPI2_HOST(`1`)      | SPI2_HOST(`1`)      | -                    |

[^1]: SPI bus attached to the flash (can use the same data lines but different SS)
[^2]: SPI bus normally mapped to pins 12 - 15 on ESP32 but can be matrixed to any pins
[^3]: SPI bus normally attached to pins 5, 18, 19, and 23 on ESP32 but can be matrixed to any pins

<details>
<summary>Reference</summary>

- https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html
- https://github.com/espressif/arduino-esp32/blob/099b432d10fb4ca1529c52241bcadcb8a4386f17/cores/esp32/esp32-hal-spi.h#L28-L37
- https://github.com/espressif/arduino-esp32/blob/099b432d10fb4ca1529c52241bcadcb8a4386f17/libraries/SPI/src/SPI.cpp#L346-L350
- https://github.com/espressif/arduino-esp32/blob/099b432d10fb4ca1529c52241bcadcb8a4386f17/cores/esp32/esp32-hal-spi.h#L28-L37
- https://github.com/espressif/arduino-esp32/blob/099b432d10fb4ca1529c52241bcadcb8a4386f17/cores/esp32/esp32-hal-spi.c#L719-L752
- https://github.com/espressif/arduino-esp32/blob/099b432d10fb4ca1529c52241bcadcb8a4386f17/tools/sdk/esp32/include/driver/include/driver/sdspi_host.h#L23-L29
- https://github.com/espressif/arduino-esp32/blob/099b432d10fb4ca1529c52241bcadcb8a4386f17/tools/sdk/esp32/include/hal/include/hal/spi_types.h#L26-L31
- https://github.com/espressif/arduino-esp32/blob/099b432d10fb4ca1529c52241bcadcb8a4386f17/tools/sdk/esp32/include/hal/include/hal/spi_types.h#L77-L87

</details>

## APIs

### Master

```C++
/// @brief initialize SPI with the default pin assignment for HSPI, FSPI or VSPI
bool begin(uint8_t spi_bus = HSPI);
/// @brief initialize SPI with HSPI/FSPI/VSPI, sck, miso, mosi, and ss pins
bool begin(uint8_t spi_bus, int sck, int miso, int mosi, int ss);
/// @brief initialize SPI with HSPI/FSPI/VSPI and Qued SPI pins
bool begin(uint8_t spi_bus, int sck, int ss, int data0, int data1, int data2, int data3);
/// @brief initialize SPI with HSPI/FSPI/VSPI and Octo SPI pins
bool begin(uint8_t spi_bus, int sck, int ss, int data0, int data1, int data2, int data3, int data4, int data5, int data6, int data7);
/// @brief stop spi master (terminate spi_master_task and deinitialize spi)
void end();

/// @brief allocate dma memory buffer (requires the memory allocated with this method for dma)
static uint8_t *allocDMABuffer(size_t n_bytes);

/// @brief execute one transaction and wait for the completion
size_t transfer(const uint8_t* tx_buf, uint8_t* rx_buf, size_t size, uint32_t timeout_ms = 0);
size_t transfer(uint8_t command_bits, uint8_t address_bits, uint8_t dummy_bits, uint32_t flags, uint16_t cmd, uint64_t addr, const uint8_t* tx_buf, uint8_t* rx_buf, size_t size, uint32_t timeout_ms);

/// @brief queue transaction to internal transaction buffer.
///        To start transaction, wait() or trigger() must be called.
bool queue(const uint8_t* tx_buf, uint8_t* rx_buf, size_t size);
bool queue(uint8_t command_bits, uint8_t address_bits, uint8_t dummy_bits, uint32_t flags, uint16_t cmd, uint64_t addr, const uint8_t* tx_buf, uint8_t* rx_buf, size_t size);

/// @brief execute queued transactions and wait for the completion.
///        rx_buf is automatically updated after the completion of each transaction.
std::vector<size_t> wait(uint32_t timeout_ms = 0);

/// @brief execute queued transactions asynchronously in the background (without blocking).
///        numBytesReceivedAll() or numBytesReceived() is required to confirm the results of transactions.
///        rx_buf is automatically updated after the completion of each transaction.
bool trigger();

/// @brief return the number of in-flight transactions
size_t numTransactionsInFlight();
/// @brief return the number of completed but not received transaction results
size_t numTransactionsCompleted();
/// @brief return the number of completed but not received transaction errors
size_t numTransactionErrors();
/// @brief return the oldest result of the completed transaction (received bytes)
size_t numBytesReceived();
/// @brief return all results of the completed transactions (received bytes)
std::vector<size_t> numBytesReceivedAll();
/// @brief return the oldest error of the completed transaction
esp_err_t error();
/// @brief return all errors of the completed transactions
std::vector<esp_err_t> errors();
/// @brief check if the queued transactions are completed and all results are handled
bool hasTransactionsCompletedAndAllResultsHandled();
/// @brief check if the queued transactions are completed
bool hasTransactionsCompletedAndAllResultsReady(size_t num_queued);

// ===== Main Configurations =====
// set these optional parameters before begin() if you want

/// @brief set spi data mode
void setDataMode(uint8_t mode);
/// @brief set spi frequency
void setFrequency(size_t freq);
/// @brief set default data io level
void setDataIODefaultLevel(bool level);
/// @brief set max transfer size in bytes
void setMaxTransferSize(size_t size);
/// @brief set queue size (default: 1)
void setQueueSize(size_t size);

// ===== Optional Configurations =====
// set these optional parameters before begin() if you want

/// @brief set default amount of bits in command phase (0-16), used when SPI_TRANS_VARIABLE_CMD is not used, otherwise ignored.
void setDefaultCommandBits(uint8_t n);
/// @brief set default amount of bits in address phase (0-64), used when SPI_TRANS_VARIABLE_ADDR is not used, otherwise ignored.
void setDefaultAddressBits(uint8_t n);
/// @brief amount of dummy bits to insert between address and data phase.
void setDefaultDummyBits(uint8_t n);
/// @brief SPI mode, representing a pair of (CPOL, CPHA) configuration: 0: (0, 0), 1: (0, 1), 2: (1, 0), 3: (1, 1)
void setSpiMode(uint8_t m);
/// @brief Select SPI clock source, SPI_CLK_SRC_DEFAULT by default.
void setClockSource(spi_clock_source_t clk_src);
/// @brief Duty cycle of positive clock, in 1/256th increments (128 = 50%/50% duty).
///        Setting this to 0 (=not setting it) is equivalent to setting this to 128.
void setDutyCyclePos(uint8_t n);
/// @brief SPI clock speed in Hz. Derived from clock_source.
void setClockSpeedHz(size_t f);
/// @brief Maximum data valid time of slave. The time required between SCLK and MISO valid,
///        including the possible clock delay from slave to master.
///        The driver uses this value to give an extra delay before the MISO is ready on the line.
///        Leave at 0 unless you know you need a delay.
///        For better timing performance at high frequency (over 8MHz), it's suggest to have the right value.
void setInputDelayNs(int n);
/// @brief Sample point tuning of spi master receiving bit.
void setSamplePoint(spi_sampling_point_t sample_point);
/// @brief Bitwise OR of SPI_DEVICE_* flags.
void setDeviceFlags(uint32_t flags);
/// @brief Callback to be called before a transmission is started.
void setPreCb(const transaction_cb_t pre_cb);
/// @brief Callback to be called after a transmission has completed.
void setPostCb(const transaction_cb_t post_cb);
/// @brief set pre callback (ISR) and its argument that are called before/after transaction started.
///        you can call this function before every transfer() / queue() to change the behavior per transaction.
///        see more details about callbacks at https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html#_CPPv4N29spi_device_interface_config_t6pre_cbE
void setUserPreCbAndArg(const spi_master_user_cb_t &cb, void *arg);
void setUserPostCbAndArg(const spi_master_user_cb_t &cb, void *arg)
```

### Slave

```C++
/// @brief initialize SPI with the default pin assignment for HSPI, or VSPI
bool begin(const uint8_t spi_bus = HSPI);
/// @brief initialize SPI with HSPI/FSPI/VSPI, sck, miso, mosi, and ss pins
bool begin(uint8_t spi_bus, int sck, int miso, int mosi, int ss);
/// @brief initialize SPI with HSPI/FSPI/VSPI and Qued SPI pins
bool begin(uint8_t spi_bus, int sck, int ss, int data0, int data1, int data2, int data3);
/// @brief initialize SPI with HSPI/FSPI/VSPI and Octo SPI pins
bool begin(uint8_t spi_bus, int sck, int ss, int data0, int data1, int data2, int data3, int data4, int data5, int data6, int data7);
/// @brief stop spi slave (terminate spi_slave_task and deinitialize spi)
void end();

/// @brief allocate dma memory buffer (requires the memory allocated with this method for dma)
static uint8_t *allocDMABuffer(size_t n_bytes);

/// @brief execute one transaction and wait for the completion
size_t transfer(const uint8_t* tx_buf, uint8_t* rx_buf, size_t size, uint32_t timeout_ms = 0);
size_t transfer(uint32_t flags, const uint8_t* tx_buf, uint8_t* rx_buf, size_t size, uint32_t timeout_ms);

/// @brief  queue transaction to internal transaction buffer.
///         To start transaction, wait() or trigger() must be called.
bool queue(const uint8_t* tx_buf, uint8_t* rx_buf, size_t size);
bool queue(uint32_t flags, const uint8_t* tx_buf, uint8_t* rx_buf, size_t size);

/// @brief execute queued transactions and wait for the completion.
///        rx_buf is automatically updated after the completion of each transaction.
std::vector<size_t> wait(uint32_t timeout_ms = 0);

/// @brief execute queued transactions asynchronously in the background (without blocking)
///        numBytesReceivedAll() or numBytesReceived() is required to confirm the results of transactions
///        rx_buf is automatically updated after the completion of each transaction.
bool trigger();

/// @brief return the number of in-flight transactions
size_t numTransactionsInFlight();
/// @brief return the number of completed but not received transaction results
size_t numTransactionsCompleted();
/// @brief return the number of completed but not received transaction errors
size_t numTransactionErrors();
/// @brief return the oldest result of the completed transaction (received bytes)
size_t numBytesReceived();
/// @brief return all results of the completed transactions (received bytes)
std::vector<size_t> numBytesReceivedAll();
/// @brief check if the queued transactions are completed and all results are handled
/// @brief return the oldest error of the completed transaction
esp_err_t error();
/// @brief return all errors of the completed transactions
std::vector<esp_err_t> errors();
bool hasTransactionsCompletedAndAllResultsHandled();
/// @brief check if the queued transactions are completed
bool hasTransactionsCompletedAndAllResultsReady(size_t num_queued);

// ===== Main Configurations =====
// set these optional parameters before begin() if you want

/// @brief set spi data mode
void setDataMode(uint8_t mode);
/// @brief set default data io level
void setDataIODefaultLevel(bool level);
/// @brief set max transfer size in bytes
void setMaxTransferSize(size_t size);
/// @brief set queue size (default: 1)
void setQueueSize(size_t size);

// ===== Optional Configurations =====
// set these optional parameters before begin() if you want

/// @brief Bitwise OR of SPI_SLAVE_* flags.
void setSlaveFlags(uint32_t flags);
/// @brief SPI mode, representing a pair of (CPOL, CPHA) configuration: 0: (0, 0), 1: (0, 1), 2: (1, 0), 3: (1, 1)
void setSpiMode(uint8_t m);
/// @brief Callback called after the SPI registers are loaded with new data.
void setPostSetupCb(const slave_transaction_cb_t &post_setup_cb);
/// @brief Callback called after a transaction is done.
void setPostTransCb(const slave_transaction_cb_t &post_trans_cb);
/// @brief set post_setup callback (ISR) and its argument that are called after transaction setup completed.
///        you can call this function before every transfer() / queue() to change the behavior per transaction.
///        see more details about callbacks at https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html#_CPPv4N29spi_device_interface_config_t6pre_cbE
void setUserPostSetupCbAndArg(const spi_slave_user_cb_t &cb, void *arg);
void setUserPostTransCbAndArg(const spi_slave_user_cb_t &cb, void *arg);
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

## Reference

- [ESP-IDF Programming Guide / API Reference / SPI Master Driver](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html#)
- [ESP-IDF Programming Guide / API Reference / SPI Slave Driver](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_slave.html#)

## License

MIT
