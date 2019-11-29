#include "ESP32DMASPISlave.h"

ARDUINO_ESP32_DMA_SPI_NAMESPACE_BEGIN


void spi_slave_setup_done(spi_slave_transaction_t* trans)
{
    // printf("[callback] SPI slave setup finished\n");
}

void spi_slave_trans_done(spi_slave_transaction_t* trans)
{
    // printf("[callback] SPI slave transaction finished\n");
    ((Slave*)trans->user)->results.push_back(trans->trans_len);
    ((Slave*)trans->user)->transactions.pop_front();
}

bool Slave::begin(uint8_t spi_bus, int8_t sck, int8_t miso, int8_t mosi, int8_t ss)
{
    if((sck == -1) && (miso == -1) && (mosi == -1) && (ss == -1))
    {
        bus_cfg.sclk_io_num = (spi_bus == VSPI) ? SCK  : 14;
        bus_cfg.miso_io_num = (spi_bus == VSPI) ? MISO : 12;
        bus_cfg.mosi_io_num = (spi_bus == VSPI) ? MOSI : 13;
        if_cfg.spics_io_num = (spi_bus == VSPI) ? SS   : 15;
    }
    else
    {
        bus_cfg.sclk_io_num = sck;
        bus_cfg.miso_io_num = miso;
        bus_cfg.mosi_io_num = mosi;
        if_cfg.spics_io_num = ss;
    }

    bus_cfg.max_transfer_sz = max_size;

    if_cfg.flags = 0; // Bitwise OR of SPI_SLAVE_* flags
    if_cfg.queue_size = queue_size;
    if_cfg.mode = mode; // must be 1 or 3 if DMA is used
    // TODO: callbacks
    if_cfg.post_setup_cb = spi_slave_setup_done;
    if_cfg.post_trans_cb = spi_slave_trans_done;


    // make sure to use DMA buffer
    if ((dma_chan != 1) && (dma_chan != 2))
    {
        printf("[WARNING] invalid DMA channel %d, force to set channel 2. make sure to select 1 or 2\n", dma_chan);
        dma_chan = 2;
    }
    if ((mode != SPI_MODE1) && (mode != SPI_MODE3))
    {
        printf("[WARNING] invalid SPI channel %d, force to set SPI_MODE3. make sure to select MODE1 or MODE3 (because of DMA)\n", mode);
        mode = SPI_MODE3;
    }

    host = (spi_bus == HSPI) ? HSPI_HOST : VSPI_HOST;
    esp_err_t e = spi_slave_initialize(host, &bus_cfg, &if_cfg, dma_chan);

    return (e == ESP_OK);
}

bool Slave::end()
{
    return (spi_slave_free(host) == ESP_OK);
}

uint8_t* Slave::allocDMABuffer(size_t s)
{
    return (uint8_t*)heap_caps_malloc(s, MALLOC_CAP_DMA);
}

bool Slave::wait(uint8_t* rx_buf, size_t size)
{
    return wait(rx_buf, NULL, size);
}

bool Slave::wait(uint8_t* rx_buf, uint8_t* tx_buf, size_t size)
{
    if (!transactions.empty())
    {
        printf("[ERROR] can not execute transfer if queued transaction exits. queueed size = %d\n", transactions.size());
        return 0;
    }

    addTransaction(rx_buf, tx_buf, size);

    esp_err_t e = spi_slave_transmit(host, &transactions.back(), portMAX_DELAY);
    if (e != ESP_OK)
    {
        printf("[ERROR] SPI device transmit failed : %d\n", e);
        transactions.pop_back();
        return 0;
    }

    return (e == ESP_OK);
}


bool Slave::queue(uint8_t* rx_buf, size_t size)
{
    return queue(rx_buf, NULL, size);
}

bool Slave::queue(uint8_t* rx_buf, uint8_t* tx_buf, size_t size)
{
    if (transactions.size() >= queue_size)
    {
        printf("[WARNING] queue is full with transactions. discard new transaction request\n");
        return false;
    }

    addTransaction(rx_buf, tx_buf, size);
    esp_err_t e = spi_slave_queue_trans(host, &transactions.back(), portMAX_DELAY);

    return (e == ESP_OK);
}

void Slave::yield()
{
    size_t n = transactions.size();
    for (uint8_t i = 0; i < n; ++i)
    {
        spi_slave_transaction_t* r_trans;
        esp_err_t e = spi_slave_get_trans_result(host, &r_trans, portMAX_DELAY);
        if (e != ESP_OK)
            printf("[ERROR] SPI slave get trans result failed %d / %d : %d\n", i, n, e);
    }
}


size_t Slave::remained() const
{
    return transactions.size();
}

size_t Slave::available() const
{
    return results.size();
}

uint32_t Slave::size() const
{
    return results.front() / 8;
}

void Slave::pop()
{
    results.pop_front();
}


void Slave::setDataMode(uint8_t m)
{
    mode = m;
}

void Slave::setMaxTransferSize(int s)
{
    max_size = s;
}

void Slave::setDMAChannel(int c)
{
    dma_chan = c;
}

void Slave::setQueueSize(int s)
{
    queue_size = s;
}


void Slave::addTransaction(uint8_t* rx_buf, uint8_t* tx_buf, size_t size)
{
    transactions.emplace_back(spi_slave_transaction_t());
    // transactions.back().cmd = ;
    // transactions.back().addr = ;
    transactions.back().length = 8 * size; // in bit size
    transactions.back().user = (void*)this;
    transactions.back().tx_buffer = tx_buf;
    transactions.back().rx_buffer = rx_buf;
}

void Slave::pushResult(uint32_t s)
{
    results.push_back(s);
}

ARDUINO_ESP32_DMA_SPI_NAMESPACE_END
