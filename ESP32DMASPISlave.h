#pragma once
#ifndef ESP32DMASPI_SLAVE_H
#define ESP32DMASPI_SLAVE_H

#include <Arduino.h>
#include <SPI.h>
#include <driver/spi_slave.h>
#include <deque>

#ifndef ARDUINO_ESP32_DMA_SPI_NAMESPACE_BEGIN
#define ARDUINO_ESP32_DMA_SPI_NAMESPACE_BEGIN \
    namespace arduino {                       \
    namespace esp32 {                         \
        namespace spi {                       \
            namespace dma {
#endif
#ifndef ARDUINO_ESP32_DMA_SPI_NAMESPACE_END
#define ARDUINO_ESP32_DMA_SPI_NAMESPACE_END \
    }                                       \
    }                                       \
    }                                       \
    }
#endif

ARDUINO_ESP32_DMA_SPI_NAMESPACE_BEGIN

void spi_slave_setup_done(spi_slave_transaction_t* trans);
void spi_slave_trans_done(spi_slave_transaction_t* trans);

class Slave {
    // callbacks
    friend void spi_slave_setup_done(spi_slave_transaction_t* trans);
    friend void spi_slave_trans_done(spi_slave_transaction_t* trans);

    spi_slave_interface_config_t if_cfg {
        .spics_io_num = 15,  // HSPI
        .flags = 0,
        .queue_size = 3,
        .mode = SPI_MODE0,
        .post_setup_cb = spi_slave_setup_done,
        .post_trans_cb = spi_slave_trans_done,
    };

    spi_bus_config_t bus_cfg {
        .mosi_io_num = 13,        // HSPI
        .miso_io_num = 12,        // HSPI
        .sclk_io_num = 14,        // HSPI
        .max_transfer_sz = 4092,  // default: 4092 if DMA enabled, SOC_SPI_MAXIMUM_BUFFER_SIZE if DMA disabled
        .flags = SPICOMMON_BUSFLAG_SLAVE,
    };

    spi_host_device_t host {HSPI_HOST};
    int dma_chan {SPI_DMA_CH_AUTO};  // must be 1, 2 or SPI_DMA_CH_AUTO

    std::deque<spi_slave_transaction_t> transactions;
    std::deque<uint32_t> results;

public:
    bool begin(const uint8_t spi_bus = HSPI) {
        bus_cfg.mosi_io_num = (spi_bus == VSPI) ? MOSI : 13;
        bus_cfg.miso_io_num = (spi_bus == VSPI) ? MISO : 12;
        bus_cfg.sclk_io_num = (spi_bus == VSPI) ? SCK : 14;
        if_cfg.spics_io_num = (spi_bus == VSPI) ? SS : 15;
        return initialize(spi_bus);
    }

    bool begin(const uint8_t spi_bus, const int8_t sck, const int8_t miso, const int8_t mosi, const int8_t ss) {
        bus_cfg.mosi_io_num = mosi;
        bus_cfg.miso_io_num = miso;
        bus_cfg.sclk_io_num = sck;
        if_cfg.spics_io_num = ss;
        return initialize(spi_bus);
    }

    bool end() {
        return (spi_slave_free(host) == ESP_OK);
    }

    uint8_t* allocDMABuffer(const size_t n) {
        if (n % 4 != 0) {
            printf("[WARN] DMA buffer size must be multiples of 4 bytes\n");
        }
        return (uint8_t*)heap_caps_malloc(n, MALLOC_CAP_DMA);
    }

    // wait for transaction one by one
    bool wait(uint8_t* rx_buf, const size_t size) {
        return wait(rx_buf, NULL, size);
    }

    bool wait(uint8_t* rx_buf, const uint8_t* tx_buf, const size_t size) {
        if (!transactions.empty()) {
            printf(
                "[WARN] cannot execute transfer if queued transaction exists. queued transactions = %d\n",
                transactions.size());
            return 0;
        }

        addTransaction(rx_buf, tx_buf, size);

        esp_err_t e = spi_slave_transmit(host, &transactions.back(), portMAX_DELAY);
        if (e != ESP_OK) {
            printf("[ERROR] SPI device transmit failed : %d\n", e);
            transactions.pop_back();
            return 0;
        }

        return (e == ESP_OK);
    }

    // queueing transaction
    // wait (blocking) and timeout occurs if queue is full with transaction
    // (but designed not to queue transaction more than queue_size, so there is no timeout argument)
    bool queue(uint8_t* rx_buf, const size_t size) {
        return queue(rx_buf, NULL, size);
    }
    bool queue(uint8_t* rx_buf, const uint8_t* tx_buf, const size_t size) {
        if (transactions.size() >= if_cfg.queue_size) {
            printf("[WARNING] slave queue is full with transactions. discard new transaction request\n");
            return false;
        }

        addTransaction(rx_buf, tx_buf, size);
        esp_err_t e = spi_slave_queue_trans(host, &transactions.back(), portMAX_DELAY);

        return (e == ESP_OK);
    }

    // wait until all queued transaction will be done by master
    // if yield is finished, all the buffer is updated to latest
    void yield() {
        size_t n = transactions.size();
        for (uint8_t i = 0; i < n; ++i) {
            spi_slave_transaction_t* r_trans;
            esp_err_t e = spi_slave_get_trans_result(host, &r_trans, portMAX_DELAY);
            if (e != ESP_OK) {
                printf("[ERROR] SPI slave get trans result failed %d / %d : %d\n", i, n, e);
            }
        }
    }

    // transaction result info
    size_t available() const {
        return results.size();
    }

    size_t remained() const {
        return transactions.size();
    }

    uint32_t size() const {
        return results.front() / 8;
    }
    void pop() {
        results.pop_front();
    }

    // ===== Main Configurations =====
    // set these optional parameters before begin() if you want

    void setDataMode(const uint8_t m) {
        setSpiMode(m);
    }

    void setMaxTransferSize(const int n) {
        bus_cfg.max_transfer_sz = n;
    }

    // ===== Optional Configurations =====

    void setSlaveFlags(const uint32_t flags) {
        if_cfg.flags = flags;
    }

    void setQueueSize(const int n) {
        if_cfg.queue_size = n;
    }

    void setSpiMode(const uint8_t m) {
        if_cfg.mode = m;
    }

    void setDMAChannel(const uint8_t c) {  // default: auto
        if ((1 <= c) && (c <= 3)) {
            dma_chan = c;
        } else {
            printf("[WARN] invalid DMA channel %d, force to set auto select. make sure to select 1 or 2\n", c);
            dma_chan = SPI_DMA_CH_AUTO;
        }
    }

private:
    bool initialize(const uint8_t spi_bus) {
        host = (spi_bus == HSPI) ? HSPI_HOST : VSPI_HOST;
        esp_err_t e = spi_slave_initialize(host, &bus_cfg, &if_cfg, dma_chan);

        return (e == ESP_OK);
    }

    void addTransaction(uint8_t* rx_buf, const uint8_t* tx_buf, const size_t size) {
        transactions.emplace_back(spi_slave_transaction_t());
        transactions.back().length = 8 * size;  // in bit size
        transactions.back().tx_buffer = tx_buf;
        transactions.back().rx_buffer = rx_buf;
        transactions.back().user = (void*)this;
    }

    void pushResult(const uint32_t s) {
        results.push_back(s);
    }
};

inline void spi_slave_setup_done(spi_slave_transaction_t* trans) {
    // printf("[callback] SPI slave setup finished\n");
}

inline void spi_slave_trans_done(spi_slave_transaction_t* trans) {
    // printf("[callback] SPI slave transaction finished\n");
    ((Slave*)trans->user)->results.push_back(trans->trans_len);
    ((Slave*)trans->user)->transactions.pop_front();
}

ARDUINO_ESP32_DMA_SPI_NAMESPACE_END

namespace ESP32DMASPI = arduino::esp32::spi::dma;

#endif  // ESP32DMASPI_SLAVE_H
