#pragma once
#ifndef ESP32DMASPI_SLAVE_H
#define ESP32DMASPI_SLAVE_H

#include <Arduino.h>
#include <SPI.h>
#include <driver/spi_slave.h>
#include <vector>

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

static constexpr const char *TAG = "ESP32DMASPISlave";
static constexpr int SPI_SLAVE_TASK_STASCK_SIZE = 1024 * 2;
static constexpr int SPI_SLAVE_TASK_PRIORITY = 5;

static constexpr int SLAVE_QUEUE_TRANS_TIMEOUT_TICKS = pdMS_TO_TICKS(5000);
static constexpr int SLAVE_GET_TRANS_RESULT_TIMEOUT_TICKS = portMAX_DELAY;

static QueueHandle_t s_trans_queue_handle {NULL};
static constexpr int SEND_TRANS_QUEUE_TIMEOUT_TICKS = pdMS_TO_TICKS(5000);
static constexpr int RECV_TRANS_QUEUE_TIMEOUT_TICKS = portMAX_DELAY;
static QueueHandle_t s_trans_result_handle {NULL};
static constexpr int SEND_TRANS_RESULT_TIMEOUT_TICKS = pdMS_TO_TICKS(5000);
static constexpr int RECV_TRANS_RESULT_TIMEOUT_TICKS = 0;
static QueueHandle_t s_in_flight_mailbox_handle {NULL};

using spi_slave_user_cb_t = std::function<void(spi_slave_transaction_t*, void*)>;

void spi_slave_post_setup_cb(spi_slave_transaction_t* trans);
void spi_slave_post_trans_cb(spi_slave_transaction_t* trans);
struct spi_slave_context_t
{
    spi_slave_interface_config_t if_cfg {
        .spics_io_num = SS,
        .flags = 0,
        .queue_size = 1,
        .mode = SPI_MODE0,
        .post_setup_cb = spi_slave_post_setup_cb,
        .post_trans_cb = spi_slave_post_trans_cb,
    };
    spi_bus_config_t bus_cfg {
        .mosi_io_num = MOSI, // union with data0_io_num
        .miso_io_num = MISO, // union with data1_io_num
        .sclk_io_num = SCK,
        .data2_io_num = -1,  // union with quadwp_io_num
        .data3_io_num = -1,  // union with quadhd_io_num
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .max_transfer_sz = 4092,  // default: 4092 if DMA enabled, SOC_SPI_MAXIMUM_BUFFER_SIZE if DMA disabled
        .flags = SPICOMMON_BUSFLAG_SLAVE,
        // .isr_cpu_id = ESP_INTR_CPU_AFFINITY_1,
        .intr_flags = 0,
    };
    spi_host_device_t host {SPI2_HOST};
    int dma_chan {SPI_DMA_CH_AUTO};  // must be 1, 2 or AUTO
};

struct spi_transaction_context_t
{
    spi_slave_transaction_t *trans;
    size_t size;
};

struct spi_slave_cb_user_context_t
{
    struct {
        spi_slave_user_cb_t user_cb;
        void *user_arg;
    } post_setup;
    struct {
        spi_slave_user_cb_t user_cb;
        void *user_arg;
    } post_trans;
};

void IRAM_ATTR spi_slave_post_setup_cb(spi_slave_transaction_t* trans)
{
    spi_slave_cb_user_context_t *user_ctx = static_cast<spi_slave_cb_user_context_t*>(trans->user);
    if (user_ctx->post_setup.user_cb) {
        user_ctx->post_setup.user_cb(trans, user_ctx->post_setup.user_arg);
    }
}

void IRAM_ATTR spi_slave_post_trans_cb(spi_slave_transaction_t* trans)
{
    spi_slave_cb_user_context_t *user_ctx = static_cast<spi_slave_cb_user_context_t*>(trans->user);
    if (user_ctx->post_trans.user_cb) {
        user_ctx->post_trans.user_cb(trans, user_ctx->post_trans.user_arg);
    }
}

void spi_slave_task(void *arg)
{
    ESP_LOGD(TAG, "spi_slave_task start");

    spi_slave_context_t *ctx = static_cast<spi_slave_context_t*>(arg);

    // initialize spi slave
    esp_err_t err = spi_slave_initialize(ctx->host, &ctx->bus_cfg, &ctx->if_cfg, ctx->dma_chan);
    assert(err == ESP_OK);

    // initialize queues
    s_trans_queue_handle = xQueueCreate(1, sizeof(spi_transaction_context_t));
    assert(s_trans_queue_handle != NULL);
    s_trans_result_handle = xQueueCreate(ctx->if_cfg.queue_size, sizeof(size_t));
    assert(s_trans_result_handle != NULL);
    s_in_flight_mailbox_handle = xQueueCreate(1, sizeof(size_t));
    assert(s_in_flight_mailbox_handle != NULL);

    // spi task
    while (true) {
        spi_transaction_context_t trans_ctx;
        if (xQueueReceive(s_trans_queue_handle, &trans_ctx, RECV_TRANS_QUEUE_TIMEOUT_TICKS)) {
            // update in-flight count
            assert(trans_ctx.trans != nullptr);
            assert(trans_ctx.size <= ctx->if_cfg.queue_size);
            xQueueOverwrite(s_in_flight_mailbox_handle, &trans_ctx.size);

            // execute new transaction if transaction request received from main task
            ESP_LOGD(TAG, "new transaction request received (size = %u)", trans_ctx.size);
            for (size_t i = 0; i < trans_ctx.size; ++i) {
                spi_slave_transaction_t *trans = &trans_ctx.trans[i];
                esp_err_t err = spi_slave_queue_trans(ctx->host, trans, SLAVE_QUEUE_TRANS_TIMEOUT_TICKS);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "failed to execute spi_slave_queue_trans(): %d", err);
                }
            }

            // wait for the completion of all of the queued transactions
            for (size_t i = 0; i < trans_ctx.size; ++i) {
                // wait for completion of next transaction
                size_t num_received_bytes = 0;
                spi_slave_transaction_t *rtrans;
                esp_err_t err = spi_slave_get_trans_result(ctx->host, &rtrans, SLAVE_GET_TRANS_RESULT_TIMEOUT_TICKS);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "failed to execute spi_device_get_trans_result(): %d", err);
                } else {
                    num_received_bytes = rtrans->trans_len / 8; // bit -> byte
                    ESP_LOGD(TAG, "transaction complete: %d bits (%d bytes) received", rtrans->trans_len, num_received_bytes);
                }

                // send the received bytes back to main task
                if (uxQueueSpacesAvailable(s_trans_result_handle) == 0) {
                    size_t discard_oldest_result = 0;
                    if (!xQueueReceive(s_trans_result_handle, &discard_oldest_result, 0)) {
                        ESP_LOGE(TAG, "failed to discard the oldest queued result");
                    }
                }
                if (!xQueueSend(s_trans_result_handle, &num_received_bytes, SEND_TRANS_RESULT_TIMEOUT_TICKS)) {
                    ESP_LOGE(TAG, "failed to send a number of received bytes to main task: %d", err);
                }

                // update in-flight count
                const size_t num_rest_in_flight = trans_ctx.size - (i + 1);
                xQueueOverwrite(s_in_flight_mailbox_handle, &num_rest_in_flight);
            }

            // should be deleted because the ownership is moved from main task
            delete[] trans_ctx.trans;

            ESP_LOGD(TAG, "all requested transactions completed");
        }

        // terminate task if requested
        if (xTaskNotifyWait(0, 0, NULL, 0) == pdTRUE) {
            break;
        }
    }

    ESP_LOGD(TAG, "terminate spi task as requested by the main task");

    vQueueDelete(s_in_flight_mailbox_handle);
    vQueueDelete(s_trans_result_handle);
    vQueueDelete(s_trans_queue_handle);

    spi_slave_free(ctx->host);

    vTaskDelete(NULL);
}

class Slave
{
    spi_slave_context_t ctx;
    std::vector<spi_slave_transaction_t> transactions;
    spi_slave_cb_user_context_t cb_user_ctx;
    TaskHandle_t spi_task_handle {NULL};

public:
    /// @brief initialize SPI with the default pin assignment for HSPI, or VSPI
    /// @param spi_bus HSPI, FSPI or VSPI
    /// @return true if initialization succeeded, false otherwise
    bool begin(const uint8_t spi_bus = HSPI)
    {
#ifdef CONFIG_IDF_TARGET_ESP32
        this->ctx.if_cfg.spics_io_num = (spi_bus == VSPI) ? SS : 15;
        this->ctx.bus_cfg.sclk_io_num = (spi_bus == VSPI) ? SCK : 14;
        this->ctx.bus_cfg.mosi_io_num = (spi_bus == VSPI) ? MOSI : 13;
        this->ctx.bus_cfg.miso_io_num = (spi_bus == VSPI) ? MISO : 12;
#endif
        return this->initialize(spi_bus);
    }
    /// @brief initialize SPI with HSPI/FSPI/VSPI, sck, miso, mosi, and ss pins
    /// @param spi_bus HSPI, FSPI or VSPI
    /// @param sck
    /// @param miso
    /// @param mosi
    /// @param ss
    /// @return true if initialization succeeded, false otherwise
    bool begin(uint8_t spi_bus, int sck, int miso, int mosi, int ss)
    {
        this->ctx.if_cfg.spics_io_num = ss;
        this->ctx.bus_cfg.sclk_io_num = sck;
        this->ctx.bus_cfg.mosi_io_num = mosi;
        this->ctx.bus_cfg.miso_io_num = miso;
        return this->initialize(spi_bus);
    }
    /// @brief initialize SPI with HSPI/FSPI/VSPI and Qued SPI pins
    /// @param spi_bus HSPI, FSPI or VSPI
    /// @param sck
    /// @param ss
    /// @param data0
    /// @param data1
    /// @param data2
    /// @param data3
    bool begin(uint8_t spi_bus, int sck, int ss, int data0, int data1, int data2, int data3)
    {
        this->ctx.if_cfg.spics_io_num = ss;
        this->ctx.bus_cfg.sclk_io_num = sck;
        this->ctx.bus_cfg.data0_io_num = data0;
        this->ctx.bus_cfg.data1_io_num = data1;
        this->ctx.bus_cfg.data2_io_num = data2;
        this->ctx.bus_cfg.data3_io_num = data3;
        return this->initialize(spi_bus);
    }
    /// @brief initialize SPI with HSPI/FSPI/VSPI and Octo SPI pins
    /// @param spi_bus HSPI, FSPI or VSPI
    /// @param sck
    /// @param ss
    /// @param data0
    /// @param data1
    /// @param data2
    /// @param data3
    /// @param data4
    /// @param data5
    /// @param data6
    /// @param data7
    bool begin(uint8_t spi_bus, int sck, int ss, int data0, int data1, int data2, int data3, int data4, int data5, int data6, int data7)
    {
        this->ctx.if_cfg.spics_io_num = ss;
        this->ctx.bus_cfg.sclk_io_num = sck;
        this->ctx.bus_cfg.data0_io_num = data0;
        this->ctx.bus_cfg.data1_io_num = data1;
        this->ctx.bus_cfg.data2_io_num = data2;
        this->ctx.bus_cfg.data3_io_num = data3;
        this->ctx.bus_cfg.data4_io_num = data4;
        this->ctx.bus_cfg.data5_io_num = data5;
        this->ctx.bus_cfg.data6_io_num = data6;
        this->ctx.bus_cfg.data7_io_num = data7;
        return this->initialize(spi_bus);
    }

    /// @brief stop spi slave (terminate spi_slave_task and deinitialize spi)
    void end()
    {
        xTaskNotifyGive(spi_task_handle);
    }

    /// @brief allocate dma memory buffer (requires the memory allocated with this method for dma)
    /// @param n_bytes the size of buffer in bytes
    /// @return pointer to the allocated dma buffer
    static uint8_t *allocDMABuffer(size_t n_bytes)
    {
        if (n_bytes % 4 != 0) {
            ESP_LOGW(TAG, "failed to allocate dma buffer: must be multiples of 4 bytes");
            return nullptr;
        }
        return static_cast<uint8_t*>(heap_caps_calloc(n_bytes, sizeof(uint8_t), MALLOC_CAP_DMA));
    }

    /// @brief execute one transaction and wait for the completion
    /// @param tx_buf pointer to the buffer of data to be sent
    /// @param rx_buf pointer to the buffer of data to be received
    /// @param size size of data to be sent
    /// @return the size of received bytes
    /// @note  this function is blocking until the completion of transmission
    size_t transfer(const uint8_t* tx_buf, uint8_t* rx_buf, size_t size, uint32_t timeout_ms = 0)
    {
        return this->transfer(0, tx_buf, rx_buf, size, timeout_ms);
    }
    /// @brief execute one transaction and wait for the completion of transmission to return the result
    /// @param flags SPI_TRANS_* flags
    /// @param tx_buf pointer to the buffer of data to be sent
    /// @param rx_buf pointer to the buffer of data to be received
    /// @param size size of data to be sent
    /// @return the size of received bytes
    /// @note  this function is blocking until the completion of transmission
    size_t transfer(
        uint32_t flags,
        const uint8_t* tx_buf,
        uint8_t* rx_buf,
        size_t size,
        uint32_t timeout_ms
    ) {
        if (!this->queue(flags, tx_buf, rx_buf, size)) {
            return 0;
        }
        const auto results = this->wait(timeout_ms);
        if (results.empty()) {
            return 0;
        } else {
            return results[results.size() - 1];
        }
    }

    /// @brief  queue transaction to internal transaction buffer.
    ///         To start transaction, wait() or trigger() must be called.
    /// @param tx_buf pointer to the buffer of data to be sent
    /// @param rx_buf pointer to the buffer of data to be received
    /// @param size size of data to be sent
    /// @return true if the transaction is queued successfully, false otherwise
    /// @note   If the size of queued transactions exceeds pre-defined queue_size,
    ///         automatically wait for the completion of transmission and results are stored in the background.
    ///         The results are cleared when the next transaction is queued.
    bool queue(const uint8_t* tx_buf, uint8_t* rx_buf, size_t size)
    {
        return this->queue(0, tx_buf, rx_buf, size);
    }
    /// @brief  queue transaction to internal transaction buffer.
    ///         To start transaction, wait() or trigger() must be called.
    /// @param flags SPI_TRANS_* flags
    /// @param tx_buf pointer to the buffer of data to be sent
    /// @param rx_buf pointer to the buffer of data to be received
    /// @param size size of data to be sent
    /// @return true if the transaction is queued successfully, false otherwise
    /// @note   If the size of queued transactions exceeds pre-defined queue_size,
    ///         automatically wait for the completion of transmission and results are stored in the background.
    ///         The results are cleared when the next transaction is queued.
    bool queue(
        uint32_t flags,
        const uint8_t* tx_buf,
        uint8_t* rx_buf,
        size_t size
    ) {
        if (size % 4 != 0) {
            ESP_LOGW(TAG, "failed to queue transaction: buffer size must be multiples of 4 bytes");
            return false;
        }
        if (this->transactions.size() >= this->ctx.if_cfg.queue_size) {
            ESP_LOGW(TAG, "failed to queue transaction: queue is full - only %u transactions can be queued at once", this->ctx.if_cfg.queue_size);
            return false;
        }
        this->queueTransaction(flags, size, tx_buf, rx_buf);
        return true;
    }

    /// @brief execute queued transactions and wait for the completion.
    ///        rx_buf is automatically updated after the completion of each transaction.
    /// @param timeout_ms timeout in milliseconds
    /// @return a vector of the received bytes for all transactions
    std::vector<size_t> wait(uint32_t timeout_ms = 0)
    {
        if (!this->trigger()) {
            return std::vector<size_t>();
        }
        return this->waitTransaction(timeout_ms);
    }

    /// @brief execute queued transactions asynchronously in the background (without blocking)
    ///        numBytesReceivedAll() or numBytesReceived() is required to confirm the results of transactions
    ///        rx_buf is automatically updated after the completion of each transaction.
    /// @return true if the transaction is queued successfully, false otherwise
    bool trigger()
    {
        spi_transaction_context_t trans_ctx {
            .trans = new spi_slave_transaction_t[this->transactions.size()],
            .size = this->transactions.size(),
        };
        for (size_t i = 0; i < this->transactions.size(); i++) {
            trans_ctx.trans[i] = std::move(this->transactions[i]);
        }
        // NOTE: spi_transaction_ext_t should be delete inside of spi task after use
        int ret = xQueueSend(s_trans_queue_handle, &trans_ctx, SEND_TRANS_QUEUE_TIMEOUT_TICKS);
        // clearing transactions is safe because data was moved to trans_ctx
        this->transactions.clear();
        if (!ret) {
            ESP_LOGE(TAG, "failed to queue transaction: transaction queue between main and spi task is full");
            return false;
        }
        return true;
    }

    /// @brief return the number of in-flight transactions
    /// @return the number of in-flight transactions
    size_t numTransactionsInFlight()
    {
        size_t num_in_flight = 0;
        xQueuePeek(s_in_flight_mailbox_handle, &num_in_flight, 0);
        return num_in_flight;
    }

    /// @brief return the number of completed but not received transaction results
    /// @return the number of completed but not received transaction results
    size_t numTransactionsCompleted()
    {
        return uxQueueMessagesWaiting(s_trans_result_handle);
    }

    /// @brief return the oldest result of the completed transaction (received bytes)
    /// @return the oldest result of the completed transaction (received bytes)
    /// @note this method pops front of the result queue
    size_t numBytesReceived()
    {
        if (this->numTransactionsCompleted() > 0) {
            size_t num_received_bytes = 0;
            if (xQueueReceive(s_trans_result_handle, &num_received_bytes, RECV_TRANS_RESULT_TIMEOUT_TICKS)) {
                return num_received_bytes;
            } else {
                ESP_LOGE(TAG, "failed to received queued result");
                return 0;
            }
        }
        return 0;
    }

    /// @brief return all results of the completed transactions (received bytes)
    /// @return all results of the completed transactions (received bytes)
    /// @note this method pops front of the result queue
    std::vector<size_t> numBytesReceivedAll()
    {
        std::vector<size_t> results;
        const size_t num_results = this->numTransactionsCompleted();
        results.reserve(num_results);
        for (size_t i = 0; i < num_results; ++i) {
            results.emplace_back(this->numBytesReceived());
        }
        return results;
    }

    // ===== Main Configurations =====
    // set these optional parameters before begin() if you want

    /// @brief set spi data mode
    /// @param mode SPI_MODE0, 1, 2, 3
    /// @note alias for setSpiMode()
    void setDataMode(uint8_t mode)
    {
        this->setSpiMode(mode);
    }

    /// @brief set max transfer size in bytes
    /// @param size max bytes to transfer
    void setMaxTransferSize(size_t size)
    {
        this->ctx.bus_cfg.max_transfer_sz = static_cast<int>(size);
    }

    /// @brief set queue size (default: 1)
    /// @param size queue size
    void setQueueSize(size_t size)
    {
        this->ctx.if_cfg.queue_size = size;
    }

#ifdef CONFIG_IDF_TARGET_ESP32
    /// @brief set dma channel to use
    /// @param dma_chan dma channel (SPI_DMA_CH1 or SPI_DMA_CH2 only)
    void setDMAChannel(spi_common_dma_t dma_chan)
    {
        if ((dma_chan == SPI_DMA_CH1) || (dma_chan == SPI_DMA_CH2) || (dma_chan == SPI_DMA_CH_AUTO)) {
            this->ctx.dma_chan = dma_chan;
        } else {
            ESP_LOGW(TAG, "invalid dma channel %d: make sure to select SPI_DMA_CH1, SPI_DMA_CH2 or SPI_DMA_CH_AUTO", dma_chan);
        }
    }
#endif

    // ===== Optional Configurations =====

    /// @brief Bitwise OR of SPI_SLAVE_* flags.
    /// @param flags
    void setSlaveFlags(uint32_t flags) { this->ctx.if_cfg.flags = flags; }

    /// @brief SPI mode, representing a pair of (CPOL, CPHA) configuration: 0: (0, 0), 1: (0, 1), 2: (1, 0), 3: (1, 1)
    /// @param n
    void setSpiMode(uint8_t m) { this->ctx.if_cfg.mode = m; }

    /// @brief Callback called after the SPI registers are loaded with new data.
    /// @param post_setup_cb
    /// @note  This callback is called within interrupt context should be in IRAM for best performance, see "Transferring Speed" section in the SPI Master documentation for full details. If not, the callback may crash during flash operation when the driver is initialized with ESP_INTR_FLAG_IRAM.
    void setPostSetupCb(const slave_transaction_cb_t &post_setup_cb) { this->ctx.if_cfg.post_setup_cb = post_setup_cb; }

    /// @brief Callback called after a transaction is done.
    /// @param post_trans_cb
    /// @note  This callback is called within interrupt context should be in IRAM for best performance, see "Transferring Speed" section in the SPI Master documentation for full details. If not, the callback may crash during flash operation when the driver is initialized with ESP_INTR_FLAG_IRAM.
    void setPostTransCb(const slave_transaction_cb_t &post_trans_cb) { this->ctx.if_cfg.post_trans_cb = post_trans_cb; }

    /// @brief set post_setup callback (ISR) and its argument that are called after transaction setup completed.
    ///        you can call this function before every transfer() / queue() to change the behavior per transaction.
    ///        see more details about callbacks at https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html#_CPPv4N29spi_device_interface_config_t6pre_cbE
    /// @param cb  callback that is called when pre callbacks are called
    /// @param arg pointer to your own data that you want to pass to the callbak
    /// @note      post_setup callbacks will be called within the interrupt context
    void setUserPostSetupCbAndArg(const spi_slave_user_cb_t &cb, void *arg)
    {
        this->cb_user_ctx.post_setup.user_cb = cb;
        this->cb_user_ctx.post_setup.user_arg = arg;
    }
    /// @brief set post_trans callback (ISR) and its argument that are called after transaction completed.
    ///        you can call this function before every transfer() / queue() to change the behavior per transaction.
    ///        see more details about callbacks at https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html#_CPPv4N29spi_device_interface_config_t6pre_cbE
    /// @param cb  callback that is called when pre/post callbacks are called
    /// @param arg pointer to your own data that you want to pass to the callbak
    /// @note      post_trans callbacks will be called within the interrupt context
    void setUserPostTransCbAndArg(const spi_slave_user_cb_t &cb, void *arg)
    {
        this->cb_user_ctx.post_trans.user_cb = cb;
        this->cb_user_ctx.post_trans.user_arg = arg;
    }

private:
    static spi_host_device_t hostFromBusNumber(uint8_t spi_bus)
    {
        switch (spi_bus) {
            case FSPI:
#ifdef CONFIG_IDF_TARGET_ESP32
                return SPI1_HOST;
#else
                return SPI2_HOST;
#endif
            case HSPI:
#if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32C3)
                return SPI2_HOST;
#else
                return SPI3_HOST;
#endif
#ifdef CONFIG_IDF_TARGET_ESP32
            case VSPI:
                return SPI3_HOST;
#endif
            default:
                return SPI2_HOST;
        }
    }

    bool initialize(const uint8_t spi_bus)
    {
        this->ctx.host = this->hostFromBusNumber(spi_bus);
        this->ctx.bus_cfg.flags |= SPICOMMON_BUSFLAG_SLAVE;
        this->transactions.reserve(this->ctx.if_cfg.queue_size);

        // create spi slave task
        std::string task_name = std::string("spi_slave_task_") + std::to_string(this->ctx.if_cfg.spics_io_num);
        int ret = xTaskCreatePinnedToCore(spi_slave_task, task_name.c_str(), SPI_SLAVE_TASK_STASCK_SIZE, static_cast<void*>(&this->ctx), SPI_SLAVE_TASK_PRIORITY, &spi_task_handle, 1);
        if (ret != pdPASS) {
            ESP_LOGE(TAG, "failed to create spi_slave_task: %d", ret);
            return false;
        }

        return true;
    }

    spi_slave_transaction_t generateTransaction(uint32_t flags, size_t size, const uint8_t* tx_buf, uint8_t* rx_buf)
    {
        spi_slave_transaction_t trans;

        // trans.flags = flags;
        trans.length = 8 * size;  // in bit size
        trans.trans_len = 0;      // will be written after transaction
        trans.tx_buffer = (tx_buf == nullptr) ? NULL : tx_buf;
        trans.rx_buffer = (rx_buf == nullptr) ? NULL : rx_buf;
        trans.user = &this->cb_user_ctx; // user-defined callback and arg

        return trans;
    }

    void queueTransaction(uint32_t flags, size_t size, const uint8_t* tx_buf, uint8_t* rx_buf)
    {
        spi_slave_transaction_t trans = generateTransaction(flags, size, tx_buf, rx_buf);
        this->transactions.push_back(std::move(trans));
    }

    std::vector<size_t> waitTransaction(uint32_t timeout_ms)
    {
        uint32_t start_ms = millis();
        while ((timeout_ms == 0) ? true : (millis() < start_ms + timeout_ms)) {
            if (this->numTransactionsInFlight() == 0) {
                return this->numBytesReceivedAll();
            }
        }
        return std::vector<size_t>();
    }
};

ARDUINO_ESP32_DMA_SPI_NAMESPACE_END

namespace ESP32DMASPI = arduino::esp32::spi::dma;

#endif  // ESP32DMASPI_SLAVE_H
