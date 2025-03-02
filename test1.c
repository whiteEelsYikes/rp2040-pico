#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"

#define I2C_PORT i2c0
#define OLED_ADDR 0x3C

void i2c_dma_write(uint8_t *data, size_t length) {
    int dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    dma_channel_configure(dma_chan, &c,
                          &i2c_get_hw(I2C_PORT)->data_cmd, // Write address
                          data, // Read address
                          length, // Number of transfers
                          true // Start immediately
    );
    dma_channel_wait_for_finish_blocking(dma_chan);
    dma_channel_unclaim(dma_chan);
}

void oled_command(uint8_t command) {
    uint8_t buf[2] = {0x00, command};
    i2c_dma_write(buf, 2);
}

void oled_init() {
    uint8_t init_cmds[] = {
        0xAE, // Display off
        0xA8, 0x3F, // Set Multiplex Ratio
        // 其他初始化命令...
        0xAF // Display on
    };
    i2c_dma_write(init_cmds, sizeof(init_cmds));
}

void oled_clear() {
    uint8_t buf[1025] = {0x40};
    for (int i = 1; i < 1025; i++) {
        buf[i] = 0x00;
    }
    i2c_dma_write(buf, sizeof(buf));
}

int main() {
    stdio_init_all();
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    oled_init();
    oled_clear();

    while (true) {
        // 您的应用程序代码...
    }
    return 0;
}
