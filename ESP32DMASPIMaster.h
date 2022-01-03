#pragma once
#ifndef ESP32DMASPI_MASTER_H
#define ESP32DMASPI_MASTER_H

#include <Arduino.h>
#include <SPI.h>
#include <driver/spi_master.h>
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

class Master {
    spi_device_interface_config_t if_cfg {
        .command_bits = 0,  // 0-16
        .address_bits = 0,  // 0-64
        .dummy_bits = 0,
        .mode = SPI_MODE0,
        .duty_cycle_pos = 128,  // default: 128
        .cs_ena_pretrans = 0,   // only for half-duplex
        .cs_ena_posttrans = 0,
        .clock_speed_hz = SPI_MASTER_FREQ_8M,
        .input_delay_ns = 0,
        .spics_io_num = 15,  // HSPI
        .flags = 0,
        .queue_size = 3,
        .pre_cb = NULL,
        .post_cb = NULL,
    };

    spi_bus_config_t bus_cfg {
        .mosi_io_num = 13,        // HSPI
        .miso_io_num = 12,        // HSPI
        .sclk_io_num = 14,        // HSPI
        .max_transfer_sz = 4092,  // default: 4092 if DMA enabled, SOC_SPI_MAXIMUM_BUFFER_SIZE if DMA disabled
        .flags = SPICOMMON_BUSFLAG_MASTER,
    };

    spi_host_device_t host {HSPI_HOST};
    spi_device_handle_t handle;
    int dma_chan {SPI_DMA_CH_AUTO};  // must be 1, 2 or SPI_DMA_CH_AUTO

    std::deque<spi_transaction_ext_t> transactions;

public:
    bool begin(const uint8_t spi_bus = HSPI) {
        bus_cfg.mosi_io_num = (spi_bus == VSPI) ? MOSI : 13;
        bus_cfg.miso_io_num = (spi_bus == VSPI) ? MISO : 12;
        bus_cfg.sclk_io_num = (spi_bus == VSPI) ? SCK : 14;
        if_cfg.spics_io_num = (spi_bus == VSPI) ? SS : 15;
        return initialize(spi_bus);
    }

    bool begin(const uint8_t spi_bus, const int8_t sck, const int8_t miso, const int8_t mosi, const int8_t ss) {
        bus_cfg.sclk_io_num = sck;
        bus_cfg.miso_io_num = miso;
        bus_cfg.mosi_io_num = mosi;
        if_cfg.spics_io_num = ss;
        return initialize(spi_bus);
    }

    bool end() {
        esp_err_t e = spi_bus_remove_device(handle);
        if (e != ESP_OK) {
            printf("[ERROR] SPI bus remove device failed : %d\n", e);
            return false;
        }

        e = spi_bus_free(host);
        if (e != ESP_OK) {
            printf("[ERROR] SPI bus free failed : %d\n", e);
            return false;
        }

        return true;
    }

    uint8_t* allocDMABuffer(const size_t n) {
        if (n % 4 != 0) {
            printf("[WARN] DMA buffer size must be multiples of 4 bytes\n");
        }
        return (uint8_t*)heap_caps_malloc(n, MALLOC_CAP_DMA);
    }

    // execute transaction and wait for transmission one by one

    size_t transfer(const uint8_t* tx_buf, const size_t size) {
        return transfer(tx_buf, NULL, size);
    }
    size_t transfer(const uint8_t* tx_buf, uint8_t* rx_buf, const size_t size) {
        return transfer(0, 0, 0, 0, 0, 0, tx_buf, rx_buf, size);
    }
    size_t transfer(const uint16_t cmd, const uint64_t addr, const uint8_t* tx_buf, const size_t size) {
        return transfer(0, 0, 0, 0, cmd, addr, tx_buf, NULL, size);
    }
    size_t transfer(
        const uint16_t cmd, const uint64_t addr, const uint8_t* tx_buf, uint8_t* rx_buf, const size_t size) {
        return transfer(0, 0, 0, 0, cmd, addr, tx_buf, rx_buf, size);
    }
    size_t transfer(
        const uint8_t command_bits,
        const uint8_t address_bits,
        const uint16_t cmd,
        const uint64_t addr,
        const uint8_t* tx_buf,
        uint8_t* rx_buf,
        const size_t size) {
        return transfer(command_bits, address_bits, 0, 0, cmd, addr, tx_buf, rx_buf, size);
    }
    size_t transfer(
        const uint8_t command_bits,
        const uint8_t address_bits,
        const uint8_t dummy_bits,
        const uint32_t flags,
        const uint16_t cmd,
        const uint64_t addr,
        const uint8_t* tx_buf,
        uint8_t* rx_buf,
        const size_t size) {
        if (!transactions.empty()) {
            printf(
                "[WARN] cannot execute transfer if queued transaction exists. queued transactions = %d\n",
                transactions.size());
            return 0;
        }
        if (size % 4 != 0) {
            printf("[WARN] DMA buffer size must be multiples of 4 bytes\n");
        }

        addTransaction(command_bits, address_bits, dummy_bits, flags, cmd, addr, size, tx_buf, rx_buf);

        // send a spi transaction, wait for it to complete, and return the result
        esp_err_t e = spi_device_transmit(handle, (spi_transaction_t*)&transactions.back());
        if (e != ESP_OK) {
            printf("[ERROR] SPI device transmit failed : %d\n", e);
            transactions.pop_back();
            return 0;
        }

        size_t len = transactions.back().base.rxlength / 8;
        transactions.pop_back();
        return len;
    }

    // queueing transaction and execute simultaneously
    // wait (blocking) and timeout occurs if queue is full with transaction
    // (but designed not to queue transaction more than queue_size, so there is no timeout argument)
    bool queue(const uint8_t* tx_buf, const size_t size) {
        return queue(tx_buf, NULL, size);
    }
    bool queue(const uint8_t* tx_buf, uint8_t* rx_buf, const size_t size) {
        return queue(0, 0, 0, 0, 0, 0, tx_buf, rx_buf, size);
    }
    bool queue(const uint16_t cmd, const uint64_t addr, const uint8_t* tx_buf, const size_t size) {
        return queue(0, 0, 0, 0, cmd, addr, tx_buf, NULL, size);
    }
    bool queue(const uint16_t cmd, const uint64_t addr, const uint8_t* tx_buf, uint8_t* rx_buf, const size_t size) {
        return queue(0, 0, 0, 0, cmd, addr, tx_buf, rx_buf, size);
    }
    bool queue(
        const uint8_t command_bits,
        const uint8_t address_bits,
        const uint16_t cmd,
        const uint64_t addr,
        const uint8_t* tx_buf,
        uint8_t* rx_buf,
        const size_t size) {
        return queue(command_bits, address_bits, 0, 0, cmd, addr, tx_buf, rx_buf, size);
    }
    bool queue(
        const uint8_t command_bits,
        const uint8_t address_bits,
        const uint8_t dummy_bits,
        const uint32_t flags,
        const uint16_t cmd,
        const uint64_t addr,
        const uint8_t* tx_buf,
        uint8_t* rx_buf,
        const size_t size) {
        if (transactions.size() >= if_cfg.queue_size) {
            printf("[WARN] master queue is full with transactions. discard new transaction request\n");
            return false;
        }
        if (size % 4 != 0) {
            printf("[WARN] DMA buffer size must be multiples of 4 bytes\n");
        }

        addTransaction(command_bits, address_bits, dummy_bits, flags, cmd, addr, size, tx_buf, rx_buf);
        esp_err_t e = spi_device_queue_trans(handle, (spi_transaction_t*)&transactions.back(), portMAX_DELAY);

        return (e == ESP_OK);
    }

    void yield() {
        const size_t n = transactions.size();
        for (uint8_t i = 0; i < n; ++i) {
            spi_transaction_t* r_trans;
            esp_err_t e = spi_device_get_trans_result(handle, &r_trans, portMAX_DELAY);
            if (e != ESP_OK) {
                printf("[ERROR] SPI device get trans result failed %d / %d : %d\n", i, n, e);
            }
            transactions.pop_front();
        }
    }

    // ===== Main Configurations =====
    // set these optional parameters before begin() if you want

    void setDataMode(const uint8_t m) {  // SPI_MODE0, 1, 2, 3
        setSpiMode(m);
    }

    void setFrequency(const size_t f) {  // up to 80 MHz
        setClockSpeedHz(f);
    }

    void setMaxTransferSize(const size_t n) {
        bus_cfg.max_transfer_sz = n;
    }

    // ===== Optional Configurations =====
    // set these optional parameters before begin() if you want

    void setCommandBits(const uint8_t n) {  // 0-16
        if_cfg.command_bits = n;
    }

    void setAddressBits(const uint8_t n) {  // 0-64
        if_cfg.address_bits = n;
    }

    void setDummyBits(const uint8_t n) {
        if_cfg.dummy_bits = n;
    }

    void setDutyCyclePos(const uint8_t n) {  // 128 for 50%
        if_cfg.duty_cycle_pos = n;
    }

    void setSpiMode(const uint8_t m) {  // SPI_MODE0, 1, 2, 3
        if_cfg.mode = m;
    }

    void setCsEnaPostTrans(const uint8_t n) {
        if_cfg.cs_ena_posttrans = n;
    }

    void setClockSpeedHz(const size_t f) {  // up to 80 MHz
        if_cfg.clock_speed_hz = f;
    }

    void setInputDelayNs(const int n) {
        if_cfg.input_delay_ns = n;
    }

    void setDeviceFlags(const uint32_t flags) {
        if_cfg.flags = flags;
    }

    void setQueueSize(const size_t n) {  // default: 3
        if_cfg.queue_size = n;
    }

    void setPreCb(const transaction_cb_t pre_cb) {
        if_cfg.pre_cb = pre_cb;
    }

    void setPostCb(const transaction_cb_t post_cb) {
        if_cfg.post_cb = post_cb;
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

        bus_cfg.flags |= SPICOMMON_BUSFLAG_MASTER;

        esp_err_t e = spi_bus_initialize(host, &bus_cfg, dma_chan);
        if (e != ESP_OK) {
            printf("[ERROR] SPI bus initialize failed : %d\n", e);
            return false;
        }

        e = spi_bus_add_device(host, &if_cfg, &handle);
        if (e != ESP_OK) {
            printf("[ERROR] SPI bus add device failed : %d\n", e);
            return false;
        }

        return true;
    }

    void addTransaction(
        const uint8_t command_bits,
        const uint8_t address_bits,
        const uint8_t dummy_bits,
        const uint32_t flags,
        const uint16_t cmd,
        const uint64_t addr,
        const size_t size,
        const uint8_t* tx_buf,
        uint8_t* rx_buf) {
        transactions.emplace_back(spi_transaction_ext_t());

        // allow variable cmd/addr/dummy bits based on spi_transaction_ext_t
        transactions.back().base.flags = flags;
        transactions.back().base.flags |= SPI_TRANS_VARIABLE_CMD;
        transactions.back().base.flags |= SPI_TRANS_VARIABLE_ADDR;
        transactions.back().base.flags |= SPI_TRANS_VARIABLE_DUMMY;

        transactions.back().base.cmd = cmd;
        transactions.back().base.addr = addr;
        transactions.back().base.length = 8 * size;  // in bit size
        transactions.back().base.rxlength = 0;       // set to same one with length
        transactions.back().base.user = NULL;
        transactions.back().base.tx_buffer = tx_buf;
        transactions.back().base.rx_buffer = rx_buf;

        transactions.back().command_bits = command_bits;
        transactions.back().address_bits = address_bits;
        transactions.back().dummy_bits = dummy_bits;
    }
};

ARDUINO_ESP32_DMA_SPI_NAMESPACE_END

namespace ESP32DMASPI = arduino::esp32::spi::dma;

#endif  // ESP32DMASPI_MASTER_H
