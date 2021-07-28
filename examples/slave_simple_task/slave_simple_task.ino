#include <ESP32DMASPISlave.h>

ESP32DMASPI::Slave slave;

static const uint32_t BUFFER_SIZE = 8192;
uint8_t* spi_slave_tx_buf;
uint8_t* spi_slave_rx_buf;

void set_buffer() {
    for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
        spi_master_tx_buf[i] = i & 0xFF;
        spi_slave_tx_buf[i] = (0xFF - i) & 0xFF;
    }
    memset(spi_master_rx_buf, 0, BUFFER_SIZE);
    memset(spi_slave_rx_buf, 0, BUFFER_SIZE);
}

constexpr uint8_t CORE_TASK_SPI_SLAVE {0};
constexpr uint8_t CORE_TASK_PROCESS_BUFFER {0};

static TaskHandle_t task_handle_wait_spi = 0;
static TaskHandle_t task_handle_process_buffer = 0;

void task_wait_spi(void* pvParameters) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        slave.wait(spi_slave_rx_buf, spi_slave_tx_buf, BUFFER_SIZE);

        xTaskNotifyGive(task_handle_process_buffer);
    }
}

void task_process_buffer(void* pvParameters) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // show received data
        for (size_t i = 0; i < BUFFER_SIZE; ++i)
            printf("%d ", spi_slave_rx_buf[i]);
        printf("\n");

        slave.pop();

        xTaskNotifyGive(task_handle_wait_spi);
    }
}

void setup() {
    Serial.begin(115200);

    // to use DMA buffer, use these methods to allocate buffer
    spi_slave_tx_buf = slave.allocDMABuffer(BUFFER_SIZE);
    spi_slave_rx_buf = slave.allocDMABuffer(BUFFER_SIZE);

    set_buffer();

    delay(5000);

    slave.setDataMode(SPI_MODE3);
    slave.setMaxTransferSize(BUFFER_SIZE);
    slave.setDMAChannel(2);  // 1 or 2 only
    slave.setQueueSize(1);   // transaction queue size
    // begin() after setting
    // HSPI = CS: 15, CLK: 14, MOSI: 13, MISO: 12
    slave.begin(HSPI);

    xTaskCreatePinnedToCore(task_wait_spi, "task_wait_spi", 2048, NULL, 2, &task_handle_wait_spi, CORE_TASK_SPI_SLAVE);
    xTaskNotifyGive(task_handle_wait_spi);

    xTaskCreatePinnedToCore(task_process_buffer, "task_process_buffer", 2048, NULL, 2, &task_handle_process_buffer, CORE_TASK_PROCESS_BUFFER);
}

void loop() {
}
