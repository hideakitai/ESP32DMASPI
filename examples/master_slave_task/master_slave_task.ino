#include <ESP32DMASPIMaster.h>
#include <ESP32DMASPISlave.h>

#include "helper.h"

ESP32DMASPI::Master master;
ESP32DMASPI::Slave slave;

static const uint32_t BUFFER_SIZE = 8192;
uint8_t* spi_master_tx_buf;
uint8_t* spi_master_rx_buf;
uint8_t* spi_slave_tx_buf;
uint8_t* spi_slave_rx_buf;

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

        printf("slave received queue = %d, size = %d\n", slave.available(), slave.size());

        if (memcmp(spi_slave_rx_buf, spi_master_tx_buf, BUFFER_SIZE)) {
            printf("[ERROR] Master -> Slave Received Data has not matched !!\n");
            cmp_bug("Received ", spi_slave_rx_buf, "Sent ", spi_master_tx_buf, BUFFER_SIZE);
        }
        if (memcmp(spi_master_rx_buf, spi_slave_tx_buf, BUFFER_SIZE)) {
            printf("ERROR: Slave -> Master Received Data has not matched !!\n");
            cmp_bug("Received ", spi_master_rx_buf, "Sent ", spi_slave_tx_buf, BUFFER_SIZE);
        }

        slave.pop();

        xTaskNotifyGive(task_handle_wait_spi);
    }
}

void setup() {
    Serial.begin(115200);

    // to use DMA buffer, use these methods to allocate buffer
    spi_master_tx_buf = master.allocDMABuffer(BUFFER_SIZE);
    spi_master_rx_buf = master.allocDMABuffer(BUFFER_SIZE);
    spi_slave_tx_buf = slave.allocDMABuffer(BUFFER_SIZE);
    spi_slave_rx_buf = slave.allocDMABuffer(BUFFER_SIZE);

    set_buffer(spi_master_tx_buf, spi_master_rx_buf, spi_slave_tx_buf, spi_slave_rx_buf, BUFFER_SIZE);
    delay(5000);

    // ===== SPI Master =====

    // master device configuration
    master.setDataMode(SPI_MODE0);           // default: SPI_MODE0
    master.setFrequency(4000000);            // default: 8MHz (too fast for bread board...)
    master.setMaxTransferSize(BUFFER_SIZE);  // default: 4092 bytes
    master.setDutyCyclePos(96);              // default: 128 (required for my bread board)

    // begin() after setting
    master.begin(VSPI);  // HSPI (CS: 15, CLK: 14, MOSI: 13, MISO: 12) -> default
                         // VSPI (CS:  5, CLK: 18, MOSI: 23, MISO: 19)

    // ===== SPI Slave =====

    // slave device configuration
    slave.setDataMode(SPI_MODE0);
    slave.setMaxTransferSize(BUFFER_SIZE);

    // begin() after setting
    slave.begin(HSPI);  // HSPI = CS: 15, CLK: 14, MOSI: 13, MISO: 12 -> default
                        // VSPI (CS:  5, CLK: 18, MOSI: 23, MISO: 19)

    // connect same name pins each other
    // CS - CS, CLK - CLK, MOSI - MOSI, MISO - MISO

    printf("Main code running on core : %d\n", xPortGetCoreID());

    xTaskCreatePinnedToCore(task_wait_spi, "task_wait_spi", 2048, NULL, 2, &task_handle_wait_spi, CORE_TASK_SPI_SLAVE);
    xTaskNotifyGive(task_handle_wait_spi);

    xTaskCreatePinnedToCore(
        task_process_buffer,
        "task_process_buffer",
        2048,
        NULL,
        2,
        &task_handle_process_buffer,
        CORE_TASK_PROCESS_BUFFER);
}

void loop() {
    // start and wait to complete transaction
    master.transfer(spi_master_tx_buf, spi_master_rx_buf, BUFFER_SIZE);

    static uint32_t prev_ms = millis();
    printf("wait for next loop.. elapsed = %ld\n", millis() - prev_ms - 2000);
    prev_ms = millis();
    delay(2000);
}
