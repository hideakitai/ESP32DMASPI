#pragma once
#ifndef ESP32DMASPI_SLAVE_H
#define ESP32DMASPI_SLAVE_H

#include <Arduino.h>
#include <SPI.h>
#include <driver/spi_slave.h>
#include <deque>

namespace arduino {
namespace esp32 {
    namespace spi {
        namespace dma {

            class Slave {
                // callbacks
                friend void spi_slave_setup_done(spi_slave_transaction_t* trans);
                friend void spi_slave_trans_done(spi_slave_transaction_t* trans);

                spi_slave_interface_config_t if_cfg;
                spi_bus_config_t bus_cfg;

                spi_host_device_t host {HSPI_HOST};
                uint8_t mode {SPI_MODE3};
                int dma_chan {2};     // must be 1 or 2
                int max_size {4094};  // default size

                std::deque<spi_slave_transaction_t> transactions;
                int queue_size {1};

                std::deque<uint32_t> results;

            public:
                bool begin(
                    const uint8_t spi_bus = HSPI,
                    const int8_t sck = -1,
                    const int8_t miso = -1,
                    const int8_t mosi = -1,
                    const int8_t ss = -1);
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

                // set these optional parameters before begin() if you want
                void setDataMode(const uint8_t m);
                void setMaxTransferSize(const int s);
                void setDMAChannel(const int c);  // 1 or 2 only
                void setQueueSize(const int s);

            private:
                void addTransaction(uint8_t* rx_buf, const uint8_t* tx_buf, const size_t size);
                void pushResult(const uint32_t s);
            };

        }  // namespace dma
    }      // namespace spi
}  // namespace esp32
}  // namespace arduino

#ifndef ARDUINO_ESP32_DMA_SPI_NAMESPACE_BEGIN
#define ARDUINO_ESP32_DMA_SPI_NAMESPACE_BEGIN \
    namespace arduino {                       \
        namespace esp32 {                     \
            namespace spi {                   \
                namespace dma {
#endif
#ifndef ARDUINO_ESP32_DMA_SPI_NAMESPACE_END
#define ARDUINO_ESP32_DMA_SPI_NAMESPACE_END \
    }                                       \
    }                                       \
    }                                       \
    }
#endif

namespace ESP32DMASPI = arduino::esp32::spi::dma;

#endif  // ESP32DMASPI_SLAVE_H
