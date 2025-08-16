#pragma once
#ifndef ESP32DMASPI_MASTER_H
#define ESP32DMASPI_MASTER_H

#include <Arduino.h>
#include <SPI.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <soc/soc_caps.h>
#include <vector>
#include <string>
#include <memory>

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

static constexpr const char *TAG = "ESP32DMASPIMaster";
static constexpr int SPI_MASTER_TASK_STASCK_SIZE = 1024 * 2;
static constexpr int SPI_MASTER_TASK_PRIORITY = 5;

static QueueHandle_t s_trans_queue_handle {NULL};
static constexpr int SEND_TRANS_QUEUE_TIMEOUT_TICKS = pdMS_TO_TICKS(5000);
static constexpr int RECV_TRANS_QUEUE_TIMEOUT_TICKS = pdMS_TO_TICKS(5000);
static QueueHandle_t s_trans_result_handle {NULL};
static constexpr int SEND_TRANS_RESULT_TIMEOUT_TICKS = pdMS_TO_TICKS(5000);
static constexpr int RECV_TRANS_RESULT_TIMEOUT_TICKS = 0;
static QueueHandle_t s_trans_error_handle {NULL};
static constexpr int SEND_TRANS_ERROR_TIMEOUT_TICKS = pdMS_TO_TICKS(5000);
static constexpr int RECV_TRANS_ERROR_TIMEOUT_TICKS = 0;
static QueueHandle_t s_in_flight_mailbox_handle {NULL};

using spi_master_user_cb_t = std::function<void(spi_transaction_t*, void*)>;

void spi_master_pre_cb(spi_transaction_t* trans);
void spi_master_post_cb(spi_transaction_t* trans);
struct spi_master_context_t
{
    spi_device_interface_config_t if_cfg {
        .command_bits = 0,  // 0-16
        .address_bits = 0,  // 0-64
        .dummy_bits = 0,
        .mode = SPI_MODE0,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 1)
        .clock_source = SPI_CLK_SRC_DEFAULT,
#endif
        .duty_cycle_pos = 128,  // default: 128
        .cs_ena_pretrans = 0,   // only for half-duplex
        .cs_ena_posttrans = 0,
        .clock_speed_hz = SPI_MASTER_FREQ_8M,
        .input_delay_ns = 0,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 1)
        .sample_point = SPI_SAMPLING_POINT_PHASE_0,
#endif
        .spics_io_num = SS,
        .flags = 0,
        .queue_size = 1,
        .pre_cb = spi_master_pre_cb,
        .post_cb = spi_master_post_cb,
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
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 1)
        .data_io_default_level = false,
#endif
        .max_transfer_sz = 4092,  // default: 4092 if DMA enabled, SOC_SPI_MAXIMUM_BUFFER_SIZE if DMA disabled
        .flags = SPICOMMON_BUSFLAG_MASTER,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
        .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
#elif ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 0)
        .isr_cpu_id = INTR_CPU_ID_AUTO,
#endif
        .intr_flags = 0,
    };
    spi_host_device_t host {SPI2_HOST};
    int dma_chan {SPI_DMA_CH_AUTO};  // must be 1, 2 or AUTO
    TaskHandle_t main_task_handle {NULL};
};

struct spi_transaction_context_t
{
    spi_transaction_ext_t *trans_ext;
    size_t size;
    bool is_continuous_transactions;
    TickType_t timeout_ticks;
};

struct spi_master_cb_user_context_t
{
    struct {
        spi_master_user_cb_t user_cb;
        void *user_arg;
    } pre;
    struct {
        spi_master_user_cb_t user_cb;
        void *user_arg;
    } post;
};

void IRAM_ATTR spi_master_pre_cb(spi_transaction_t* trans)
{
    spi_master_cb_user_context_t *user_ctx = static_cast<spi_master_cb_user_context_t*>(trans->user);
    if (user_ctx->pre.user_cb) {
        user_ctx->pre.user_cb(trans, user_ctx->pre.user_arg);
    }
}

void IRAM_ATTR spi_master_post_cb(spi_transaction_t* trans)
{
    spi_master_cb_user_context_t *user_ctx = static_cast<spi_master_cb_user_context_t*>(trans->user);
    if (user_ctx->post.user_cb) {
        user_ctx->post.user_cb(trans, user_ctx->post.user_arg);
    }
}

void spi_master_task(void *arg)
{
    ESP_LOGD(TAG, "spi_master_task start");

    spi_master_context_t *ctx = static_cast<spi_master_context_t*>(arg);

    // initialize spi bus
    esp_err_t err = spi_bus_initialize(ctx->host, &ctx->bus_cfg, ctx->dma_chan);
    assert(err == ESP_OK);

    // add spi device
    spi_device_handle_t device_handle;
    err = spi_bus_add_device(ctx->host, &ctx->if_cfg, &device_handle);
    assert(err == ESP_OK);

    // initialize queues
    s_trans_queue_handle = xQueueCreate(1, sizeof(spi_transaction_context_t));
    assert(s_trans_queue_handle != NULL);
    s_trans_result_handle = xQueueCreate(ctx->if_cfg.queue_size, sizeof(size_t));
    assert(s_trans_result_handle != NULL);
    s_trans_error_handle = xQueueCreate(ctx->if_cfg.queue_size, sizeof(esp_err_t));
    assert(s_trans_error_handle != NULL);
    s_in_flight_mailbox_handle = xQueueCreate(1, sizeof(size_t));
    assert(s_in_flight_mailbox_handle != NULL);

    // spi task
    while (true) {
        spi_transaction_context_t trans_ctx;
        if (xQueueReceive(s_trans_queue_handle, &trans_ctx, RECV_TRANS_QUEUE_TIMEOUT_TICKS)) {
            // update in-flight count
            assert(trans_ctx.trans_ext != nullptr);
            assert(trans_ctx.size <= ctx->if_cfg.queue_size);
            xQueueOverwrite(s_in_flight_mailbox_handle, &trans_ctx.size);

            // begin continuous transactions if requested
            if (trans_ctx.is_continuous_transactions) {
                spi_device_acquire_bus(device_handle, portMAX_DELAY);
            }

            // execute new transaction if transaction request received from main task
            ESP_LOGD(TAG, "new transaction request received (size = %u)", trans_ctx.size);
            std::vector<esp_err_t> errs;
            errs.reserve(trans_ctx.size);
            for (size_t i = 0; i < trans_ctx.size; ++i) {
                if (trans_ctx.is_continuous_transactions && i + 1 < trans_ctx.size) {
                    trans_ctx.trans_ext[i].base.flags |= SPI_TRANS_CS_KEEP_ACTIVE;
                }
                spi_transaction_t *trans = (spi_transaction_t*)(&trans_ctx.trans_ext[i]);
                esp_err_t err = spi_device_queue_trans(device_handle, trans, trans_ctx.timeout_ticks);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "failed to execute spi_device_queue_trans(): 0x%X", err);
                }
                errs.push_back(err);
            }

            // wait for the completion of all of the queued transactions
            // reset result/error queue first
            xQueueReset(s_trans_result_handle);
            xQueueReset(s_trans_error_handle);
            for (size_t i = 0; i < trans_ctx.size; ++i) {
                // wait for completion of next transaction
                size_t num_received_bytes = 0;
                if (errs[i] == ESP_OK) {
                    spi_transaction_t *rtrans;
                    esp_err_t err = spi_device_get_trans_result(device_handle, &rtrans, trans_ctx.timeout_ticks);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "failed to execute spi_device_get_trans_result(): 0x%X", err);
                    } else {
                        num_received_bytes = rtrans->rxlength / 8; // bit -> byte
                        ESP_LOGD(TAG, "transaction complete: %d bits (%d bytes) received", rtrans->rxlength, num_received_bytes);
                    }
                } else {
                    ESP_LOGE(TAG, "skip spi_device_get_trans_result() because queue was failed: index = %u", i);
                }

                // send the received bytes back to main task
                if (!xQueueSend(s_trans_result_handle, &num_received_bytes, SEND_TRANS_RESULT_TIMEOUT_TICKS)) {
                    ESP_LOGE(TAG, "failed to send a number of received bytes to main task: %d", err);
                }
                // send the transaction error back to main task
                if (!xQueueSend(s_trans_error_handle, &errs[i], SEND_TRANS_ERROR_TIMEOUT_TICKS)) {
                    ESP_LOGE(TAG, "failed to send a transaction error to main task: %d", err);
                }

                // update in-flight count
                const size_t num_rest_in_flight = trans_ctx.size - (i + 1);
                xQueueOverwrite(s_in_flight_mailbox_handle, &num_rest_in_flight);
            }

            // end continuous transactions if requested
            if (trans_ctx.is_continuous_transactions) {
                spi_device_release_bus(device_handle);
            }

            // should be deleted because the ownership is moved from main task
            delete[] trans_ctx.trans_ext;

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
    vQueueDelete(s_trans_error_handle);
    vQueueDelete(s_trans_queue_handle);

    spi_bus_remove_device(device_handle);
    spi_bus_free(ctx->host);

    xTaskNotifyGive(ctx->main_task_handle);
    ESP_LOGD(TAG, "spi_master_task finished");

    vTaskDelete(NULL);
}

class Master
{
    spi_master_context_t ctx;
    std::vector<spi_transaction_ext_t> transactions;
    spi_master_cb_user_context_t cb_user_ctx;
    TaskHandle_t spi_task_handle {NULL};
    bool is_continuous_transactions {false};

public:
    /// @brief initialize SPI with the default pin assignment for HSPI, FSPI or VSPI
    /// @param spi_bus HSPI, FSPI or VSPI
    /// @return true if initialization succeeded, false otherwise
    bool begin(uint8_t spi_bus = HSPI)
    {
#ifdef CONFIG_IDF_TARGET_ESP32
        this->ctx.if_cfg.spics_io_num = (spi_bus == VSPI) ? SS : 15;
        this->ctx.bus_cfg.sclk_io_num = (spi_bus == VSPI) ? SCK : 14;
        this->ctx.bus_cfg.mosi_io_num = (spi_bus == VSPI) ? MOSI : 13;
        this->ctx.bus_cfg.miso_io_num = (spi_bus == VSPI) ? MISO : 12;
#else
        this->ctx.if_cfg.spics_io_num = SS;
        this->ctx.bus_cfg.sclk_io_num = SCK;
        this->ctx.bus_cfg.mosi_io_num = MOSI;
        this->ctx.bus_cfg.miso_io_num = MISO;
#endif
        return this->initialize(spi_bus);
    }
    /// @brief initialize SPI with HSPI/FSPI/VSPI, sck, miso, mosi, and ss pins
    /// @param spi_bus HSPI or VSPI (or FSIP)
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
    /// @note if you want quad spi with WP and HD pins, please use data0 as MOSI, data1 as MISO, data2 as WP, and data3 as HD
    bool begin(uint8_t spi_bus, int sck, int ss, int data0, int data1, int data2, int data3)
    {
        this->ctx.if_cfg.spics_io_num = ss;
        this->ctx.bus_cfg.sclk_io_num = sck;
        this->ctx.bus_cfg.data0_io_num = data0;
        this->ctx.bus_cfg.data1_io_num = data1;
        this->ctx.bus_cfg.data2_io_num = data2;
        this->ctx.bus_cfg.data3_io_num = data3;
        this->ctx.bus_cfg.flags |= SPICOMMON_BUSFLAG_QUAD;
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
        this->ctx.bus_cfg.flags |= SPICOMMON_BUSFLAG_OCTAL;
        return this->initialize(spi_bus);
    }

    /// @brief stop spi master (terminate spi_master_task and deinitialize spi)
    void end()
    {
        if (this->spi_task_handle == NULL) {
            ESP_LOGW(TAG, "spi_master_task already terminated");
            return;
        }
        xTaskNotifyGive(this->spi_task_handle);
        if (xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(5000)) != pdTRUE) {
            ESP_LOGW(TAG, "timeout waiting for the termination of spi_master_task");
        }
        this->spi_task_handle = NULL;
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
    /// @param timeout_ms timeout in milliseconds
    /// @return the size of received bytes
    /// @note  this function is blocking until the completion of transmission
    size_t transfer(const uint8_t* tx_buf, uint8_t* rx_buf, size_t size, uint32_t timeout_ms = 0)
    {
        return this->transfer(0, 0, 0, 0, 0, 0, tx_buf, rx_buf, size, timeout_ms);
    }
    /// @brief execute one transaction and wait for the completion of transmission to return the result
    /// @param command_bits number of bits in command phase (0-16)
    /// @param address_bits number of bits in address phase (0-64)
    /// @param dummy_bits number of dummy bits to insert between address and data phase (0-64)
    /// @param flags SPI_TRANS_* flags
    /// @param cmd command to send
    /// @param addr address to send
    /// @param tx_buf pointer to the buffer of data to be sent
    /// @param rx_buf pointer to the buffer of data to be received
    /// @param size size of data to be sent
    /// @param timeout_ms timeout in milliseconds
    /// @return the size of received bytes
    /// @note  this function is blocking until the completion of transmission
    size_t transfer(
        uint8_t command_bits,
        uint8_t address_bits,
        uint8_t dummy_bits,
        uint32_t flags,
        uint16_t cmd,
        uint64_t addr,
        const uint8_t* tx_buf,
        uint8_t* rx_buf,
        size_t size,
        uint32_t timeout_ms
    ) {
        if (!this->queue(command_bits, address_bits, dummy_bits, flags, cmd, addr, tx_buf, rx_buf, size)) {
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
    bool queue(const uint8_t* tx_buf, uint8_t* rx_buf, size_t size)
    {
        return this->queue(0, 0, 0, 0, 0, 0, tx_buf, rx_buf, size);
    }
    /// @brief  queue transaction to internal transaction buffer.
    ///         To start transaction, wait() or trigger() must be called.
    /// @param command_bits number of bits in command phase (0-16)
    /// @param address_bits number of bits in address phase (0-64)
    /// @param dummy_bits number of dummy bits to insert between address and data phase (0-64)
    /// @param flags SPI_TRANS_* flags
    /// @param cmd command to send
    /// @param addr address to send
    /// @param tx_buf pointer to the buffer of data to be sent
    /// @param rx_buf pointer to the buffer of data to be received
    /// @param size size of data to be sent
    /// @return true if the transaction is queued successfully, false otherwise
    bool queue(
        uint8_t command_bits,
        uint8_t address_bits,
        uint8_t dummy_bits,
        uint32_t flags,
        uint16_t cmd,
        uint64_t addr,
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
        this->queueTransaction(command_bits, address_bits, dummy_bits, flags, cmd, addr, size, tx_buf, rx_buf);
        return true;
    }

    /// @brief execute queued transactions and wait for the completion.
    ///        rx_buf is automatically updated after the completion of each transaction.
    /// @param timeout_ms timeout in milliseconds
    /// @return a vector of the received bytes for all transactions
    std::vector<size_t> wait(uint32_t timeout_ms = 0)
    {
        size_t num_will_be_queued = this->transactions.size();
        if (!this->trigger(timeout_ms)) {
            return std::vector<size_t>();
        }
        return this->waitTransaction(num_will_be_queued);
    }

    /// @brief execute queued transactions asynchronously in the background (without blocking)
    ///        numBytesReceivedAll() or numBytesReceived() is required to confirm the results of transactions
    ///        rx_buf is automatically updated after the completion of each transaction.
    /// @param timeout_ms timeout in milliseconds
    /// @return true if the transaction is queued successfully, false otherwise
    bool trigger(uint32_t timeout_ms = 0)
    {
        if (this->transactions.empty()) {
            ESP_LOGW(TAG, "failed to trigger transaction: no transaction is queued");
            return false;
        }
        if (this->numTransactionsInFlight() > 0) {
            ESP_LOGW(TAG, "failed to trigger transaction: there are already in-flight transactions");
            return false;
        }

        spi_transaction_context_t trans_ctx {
            .trans_ext = new spi_transaction_ext_t[this->transactions.size()],
            .size = this->transactions.size(),
            .is_continuous_transactions = this->is_continuous_transactions,
            .timeout_ticks = timeout_ms == 0 ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms),
        };
        for (size_t i = 0; i < this->transactions.size(); i++) {
            trans_ctx.trans_ext[i] = std::move(this->transactions[i]);
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

    /// @brief return the number of completed but not received transaction errors
    /// @return the number of completed but not received transaction errors
    size_t numTransactionErrors()
    {
        return uxQueueMessagesWaiting(s_trans_error_handle);
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

    /// @brief return the oldest error of the completed transaction
    /// @return the oldest error of the completed transaction
    /// @note this method pops front of the error queue
    esp_err_t error()
    {
        if (this->numTransactionErrors() > 0) {
            esp_err_t err;
            if (xQueueReceive(s_trans_error_handle, &err, RECV_TRANS_ERROR_TIMEOUT_TICKS)) {
                return err;
            } else {
                ESP_LOGE(TAG, "failed to receive queued error");
                return ESP_FAIL;
            }
        }
        return ESP_OK;
    }

    /// @brief return all errors of the completed transactions
    /// @return all errors of the completed transactions
    /// @note this method pops front of the error queue
    std::vector<esp_err_t> errors()
    {
        std::vector<esp_err_t> errs;
        const size_t num_errs = this->numTransactionErrors();
        errs.reserve(num_errs);
        for (size_t i = 0; i < num_errs; ++i) {
            errs.emplace_back(this->error());
        }
        return errs;
    }

    /// @brief check if the queued transactions are completed and all results are handled
    /// @return true if the queued transactions are completed and all results are handled, false otherwise
    bool hasTransactionsCompletedAndAllResultsHandled()
    {
        return this->numTransactionsInFlight() == 0 && this->numTransactionsCompleted() == 0;
    }

    /// @brief check if the queued transactions are completed
    /// @param num_queued the number of queued transactions
    /// @return true if the queued transactions are completed, false otherwise
    bool hasTransactionsCompletedAndAllResultsReady(size_t num_queued)
    {
        return this->numTransactionsInFlight() == 0 && this->numTransactionsCompleted() == num_queued;
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

    /// @brief set spi frequency
    /// @param freq frequency [Hz] (up to 80 [MHz])
    /// @note alias for setClockSpeedHz()
    void setFrequency(size_t freq)
    {
        this->setClockSpeedHz(freq);
    }

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 1)
    /// @brief set default data io level
    /// @param level default data io level
    void setDataIODefaultLevel(bool level)
    {
        this->ctx.bus_cfg.data_io_default_level = level;
    }
#endif

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
    // set these optional parameters before begin() if you want

    /// @brief set default amount of bits in command phase (0-16), used when SPI_TRANS_VARIABLE_CMD is not used, otherwise ignored.
    /// @param n
    void setDefaultCommandBits(uint8_t n) { this->ctx.if_cfg.command_bits = n; }

    /// @brief set default amount of bits in address phase (0-64), used when SPI_TRANS_VARIABLE_ADDR is not used, otherwise ignored.
    /// @param n
    void setDefaultAddressBits(uint8_t n) { this->ctx.if_cfg.address_bits = n; }

    /// @brief amount of dummy bits to insert between address and data phase.
    /// @param n
    void setDefaultDummyBits(uint8_t n) { this->ctx.if_cfg.dummy_bits = n; }

    /// @brief SPI mode, representing a pair of (CPOL, CPHA) configuration: 0: (0, 0), 1: (0, 1), 2: (1, 0), 3: (1, 1)
    /// @param n
    void setSpiMode(uint8_t m) { this->ctx.if_cfg.mode = m; }

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 1)
    /// @brief Select SPI clock source, SPI_CLK_SRC_DEFAULT by default.
    /// @param clk_src
    void setClockSource(spi_clock_source_t clk_src)
    {
        this->ctx.if_cfg.clock_source = clk_src;
    }
#endif

    /// @brief Duty cycle of positive clock, in 1/256th increments (128 = 50%/50% duty). Setting this to 0 (=not setting it) is equivalent to setting this to 128.
    /// @param n
    void setDutyCyclePos(uint8_t n) { this->ctx.if_cfg.duty_cycle_pos = n; }

    /// @brief SPI clock speed in Hz. Derived from clock_source.
    /// @param f
    void setClockSpeedHz(size_t f) { this->ctx.if_cfg.clock_speed_hz = f; }

    /// @brief Maximum data valid time of slave. The time required between SCLK and MISO valid, including the possible clock delay from slave to master. The driver uses this value to give an extra delay before the MISO is ready on the line. Leave at 0 unless you know you need a delay. For better timing performance at high frequency (over 8MHz), it's suggest to have the right value.
    /// @param n
    void setInputDelayNs(int n) { this->ctx.if_cfg.input_delay_ns = n; }

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 1)
    /// @brief Sample point tuning of spi master receiving bit.
    /// @param sample_point
    void setSamplePoint(spi_sampling_point_t sample_point)
    {
        this->ctx.if_cfg.sample_point = sample_point;
    }
#endif

    /// @brief Bitwise OR of SPI_DEVICE_* flags.
    /// @param flags
    void setDeviceFlags(uint32_t flags) { this->ctx.if_cfg.flags = flags; }

    /// @brief Callback to be called before a transmission is started.
    /// @param pre_cb
    /// @note  This callback is called within interrupt context should be in IRAM for best performance, see "Transferring Speed" section in the SPI Master documentation for full details. If not, the callback may crash during flash operation when the driver is initialized with ESP_INTR_FLAG_IRAM.
    void setPreCb(const transaction_cb_t pre_cb) { this->ctx.if_cfg.pre_cb = pre_cb; }

    /// @brief Callback to be called after a transmission has completed.
    /// @param post_cb
    /// @note  This callback is called within interrupt context should be in IRAM for best performance, see "Transferring Speed" section in the SPI Master documentation for full details. If not, the callback may crash during flash operation when the driver is initialized with ESP_INTR_FLAG_IRAM.
    void setPostCb(const transaction_cb_t post_cb) { this->ctx.if_cfg.post_cb = post_cb; }

    /// @brief set pre callback (ISR) and its argument that are called before transaction started.
    ///        you can call this function before every transfer() / queue() to change the behavior per transaction.
    ///        see more details about callbacks at https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html#_CPPv4N29spi_device_interface_config_t6pre_cbE
    /// @param cb  callback that is called when pre callbacks are called
    /// @param arg pointer to your own data that you want to pass to the callbak
    /// @note      pre callbacks will be called within the interrupt context
    void setUserPreCbAndArg(const spi_master_user_cb_t &cb, void *arg)
    {
        this->cb_user_ctx.pre.user_cb = cb;
        this->cb_user_ctx.pre.user_arg = arg;
    }
    /// @brief set post callback (ISR) and its argument that are called after transaction completed.
    ///        you can call this function before every transfer() / queue() to change the behavior per transaction.
    ///        see more details about callbacks at https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html#_CPPv4N29spi_device_interface_config_t6pre_cbE
    /// @param cb  callback that is called when pre/post callbacks are called
    /// @param arg pointer to your own data that you want to pass to the callbak
    /// @note      post callbacks will be called within the interrupt context
    void setUserPostCbAndArg(const spi_master_user_cb_t &cb, void *arg)
    {
        this->cb_user_ctx.post.user_cb = cb;
        this->cb_user_ctx.post.user_arg = arg;
    }

    /// @brief enable continuous transactions in the next sequence of transactions
    void enableContinuousTransactions() {
        this->is_continuous_transactions = true;
    }

    /// @brief disable continuous transactions in the next sequence of transactions
    void disableContinuousTransactions() {
        this->is_continuous_transactions = false;
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

    bool initialize(uint8_t spi_bus)
    {
        this->ctx.host = this->hostFromBusNumber(spi_bus);
        this->ctx.bus_cfg.flags |= SPICOMMON_BUSFLAG_MASTER;
        this->ctx.main_task_handle = xTaskGetCurrentTaskHandle();
        this->transactions.reserve(this->ctx.if_cfg.queue_size);

        // create spi master task
        std::string task_name = std::string("spi_master_task_") + std::to_string(this->ctx.if_cfg.spics_io_num);
#if SOC_CPU_CORES_NUM == 1
        int ret = xTaskCreatePinnedToCore(spi_master_task, task_name.c_str(), SPI_MASTER_TASK_STASCK_SIZE, static_cast<void*>(&this->ctx), SPI_MASTER_TASK_PRIORITY, &this->spi_task_handle, 0);
#else
        int ret = xTaskCreatePinnedToCore(spi_master_task, task_name.c_str(), SPI_MASTER_TASK_STASCK_SIZE, static_cast<void*>(&this->ctx), SPI_MASTER_TASK_PRIORITY, &this->spi_task_handle, 1);
#endif
        if (ret != pdPASS) {
            ESP_LOGE(TAG, "failed to create spi_master_task: %d", ret);
            return false;
        }

        return true;
    }

    spi_transaction_ext_t generateTransaction(
        uint8_t command_bits,
        uint8_t address_bits,
        uint8_t dummy_bits,
        uint32_t flags,
        uint16_t cmd,
        uint64_t addr,
        size_t size,
        const uint8_t* tx_buf,
        uint8_t* rx_buf
    ) {
        spi_transaction_ext_t trans;

        trans.base.flags = flags;
        if (this->ctx.bus_cfg.flags & SPICOMMON_BUSFLAG_DUAL) {
            trans.base.flags |= SPI_TRANS_MODE_DIO;
        } else if (this->ctx.bus_cfg.flags & SPICOMMON_BUSFLAG_QUAD) {
            trans.base.flags |= SPI_TRANS_MODE_QIO;
        } else if (this->ctx.bus_cfg.flags & SPICOMMON_BUSFLAG_OCTAL) {
            trans.base.flags |= SPI_TRANS_MODE_OCT;
        }
        // allow variable cmd/addr/dummy bits based on spi_transaction_ext_t
        trans.base.cmd = cmd;
        trans.base.addr = addr;
        trans.base.length = 8 * size;  // in bit size
        trans.base.rxlength = 0; // set to same one with length
        trans.base.user = &this->cb_user_ctx; // user-defined callback and arg
        trans.base.tx_buffer = (tx_buf == nullptr) ? NULL : tx_buf;
        trans.base.rx_buffer = (rx_buf == nullptr) ? NULL : rx_buf;

        trans.command_bits = command_bits;
        trans.address_bits = address_bits;
        trans.dummy_bits = dummy_bits;

        return trans;
    }

    void queueTransaction(
        uint8_t command_bits,
        uint8_t address_bits,
        uint8_t dummy_bits,
        uint32_t flags,
        uint16_t cmd,
        uint64_t addr,
        size_t size,
        const uint8_t* tx_buf,
        uint8_t* rx_buf
    ) {
        spi_transaction_ext_t trans_ext = generateTransaction(command_bits, address_bits, dummy_bits, flags, cmd, addr, size, tx_buf, rx_buf);
        this->transactions.push_back(std::move(trans_ext));
    }

    std::vector<size_t> waitTransaction(size_t num_will_be_queued)
    {
        // transactions inside of spi task will be timeout if failed in the background
        while (!this->hasTransactionsCompletedAndAllResultsReady(num_will_be_queued)) {
            vTaskDelay(1);
        }
        return this->numBytesReceivedAll();
    }
};

ARDUINO_ESP32_DMA_SPI_NAMESPACE_END

namespace ESP32DMASPI = arduino::esp32::spi::dma;

#endif  // ESP32DMASPI_MASTER_H
