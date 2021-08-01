#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/i2s.h"
#include "soc/gpio_sig_map.h"
#include "codec_es8388.h"
#include "math.h"

#define I2C_MASTER_NUM	0
#define ACK_CHECK_EN 0x1
#define ACK_CHECK_DIS 0x0
#define ACK_VAL I2C_MASTER_ACK
#define NACK_VAL I2C_MASTER_NACK

#define ES8388_ADDR 0b0010000

#define VOL_DEFAULT 70


#define SAMPLE_RATE     (44100)
#define I2S_NUM         (0)
#define WAVE_FREQ_HZ    (1200)
#define PI              (3.14159265)


#define I2S_BCK_IO      (GPIO_NUM_5)
#define I2S_WS_IO       (GPIO_NUM_18)
#define I2S_DO_IO       (GPIO_NUM_21)
#define I2S_DI_IO       (GPIO_NUM_19)
#define I2C_MASTER_SDA_IO 22
#define I2C_MASTER_SCL_IO 23
#define IS2_MCLK_PIN	(GPIO_NUM_3)


/*
#define I2S_BCK_IO      (GPIO_NUM_5)
#define I2S_WS_IO       (GPIO_NUM_25)
#define I2S_DO_IO       (GPIO_NUM_26)
#define I2S_DI_IO       (GPIO_NUM_35)
#define I2C_MASTER_SDA_IO 18
#define I2C_MASTER_SCL_IO 23
#define IS2_MCLK_PIN	(GPIO_NUM_0)
*/

#define SAMPLE_PER_CYCLE (SAMPLE_RATE/WAVE_FREQ_HZ)

#define SAMPLES			SAMPLE_PER_CYCLE	// Total number of samples left and right
#define	BUF_SAMPLES		SAMPLES * 4			// Size of DMA tx/rx buffer samples * left/right * 2 for 32 bit samples

// DMA Buffers
uint16_t rxBuf[BUF_SAMPLES];
uint16_t txBuf[BUF_SAMPLES];


static const char *ES_TAG = "ES8388_DRIVER";


esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

uint8_t i2c_write_bulk( uint8_t i2c_bus_addr, uint8_t reg, uint8_t bytes, uint8_t *data)
{
    printf( "Writing [%02x]=", reg );
    for ( int i = 0 ; i < bytes ; i++ )
        printf( "%02x:", data[i] );
    printf( "\n");

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, i2c_bus_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write(cmd, &reg, 1, ACK_CHECK_EN);
    i2c_master_write(cmd, data, bytes, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int ret = i2c_master_cmd_begin( I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }

    return 0;
}

uint8_t i2c_write( uint8_t i2c_bus_addr, uint8_t reg, uint8_t value)
{
    return i2c_write_bulk( i2c_bus_addr, reg, 1, &value );
}



uint8_t i2c_read( uint8_t i2c_bus_addr, uint8_t reg)
{
	   	uint8_t buffer[2];
	    //printf( "Addr: [%d] Reading register: [%d]\n", i2c_bus_addr, reg );

	    buffer[0] = reg;

	    int ret;
	    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	    // Write the register address to be read
	    i2c_master_start(cmd);
	    i2c_master_write_byte(cmd, i2c_bus_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
	    i2c_master_write_byte(cmd, buffer[0], ACK_CHECK_EN);

	    // Read the data for the register from the slave
	    i2c_master_start(cmd);
	    i2c_master_write_byte(cmd, i2c_bus_addr << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
	    i2c_master_read_byte(cmd, &buffer[0], NACK_VAL);
	    i2c_master_stop(cmd);

	    ret = i2c_master_cmd_begin( I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	    i2c_cmd_link_delete(cmd);

	    printf( "Read: [%02x]=[%02x]\n", reg, buffer[0] );

	    return (buffer[0]);
}


static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

static esp_err_t es_write_reg(uint8_t slave_addr, uint8_t reg_add, uint8_t data)
{
    return i2c_write( ES8388_ADDR, reg_add, data );
}

static esp_err_t es_read_reg(uint8_t reg_add, uint8_t *p_data)
{
    *p_data = i2c_read( ES8388_ADDR, reg_add );
    return ESP_OK;
}

void es8388_read_all()
{
	printf( "\n\n===================\n\n");
    for (int i = 0; i < 50; i++) {
        uint8_t reg = 0;
        es_read_reg(i, &reg);
    }
	printf( "\n\n===================\n\n");
}



static int es8388_set_adc_dac_volume(int mode, int volume, int dot)
{
    int res = 0;
    if ( volume < -96 || volume > 0 ) {
        ESP_LOGW(ES_TAG, "Warning: volume < -96! or > 0!\n");
        if (volume < -96)
            volume = -96;
        else
            volume = 0;
    }
    dot = (dot >= 5 ? 1 : 0);
    volume = (-volume << 1) + dot;
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL8, volume);
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL9, volume);  //ADC Right Volume=0db
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL5, volume);
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL4, volume);
    }
    return res;
}

esp_err_t es8388_init( es_dac_output_t output, es_adc_input_t input )
{
    int res = 0;

    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL3, 0x04);  // 0x04 mute/0x00 unmute&ramp;DAC unmute and  disabled digital volume control soft ramp

    res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL2, 0x50);
    res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0x00); //normal all and power up all
    res |= es_write_reg(ES8388_ADDR, ES8388_MASTERMODE, ES_MODE_SLAVE ); //CODEC IN I2S SLAVE MODE

    res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0xC0);  //disable DAC and disable Lout/Rout/1/2
    res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL1, 0x12);  //Enfr=0,Play&Record Mode,(0x17-both of mic&paly)
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL1, 0x18);//1a 0x18:16bit iis , 0x00:24
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL2, 0x02);  //DACFsMode,SINGLE SPEED; DACFsRatio,256
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL16, 0x00); // 0x00 audio on LIN1&RIN1,  0x09 LIN2&RIN2
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL17, 0x90); // only left DAC to left mixer enable 0db
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL20, 0x90); // only right DAC to right mixer enable 0db
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x80); //set internal ADC and DAC use the same LRCK clock, ADC LRCK as internal LRCK
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL23, 0x00);   //vroi=0
    res |= es8388_set_adc_dac_volume(ES_MODULE_DAC, 0, 0);          // 0db

    ESP_LOGE(ES_TAG, "Setting DAC Output: %02x", output );
    res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, output );
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0xFF);
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL1, 0xbb); // MIC Left and Right channel PGA gain


    ESP_LOGE(ES_TAG, "Setting ADC Input: %02x", input );
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL2, input);

    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL3, 0x02);
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL4, 0x0d); // Left/Right data, Left/Right justified mode, Bits length, I2S format
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL5, 0x02);  //ADCFsMode,singel SPEED,RATIO=256
    //ALC for Microphone
    res |= es8388_set_adc_dac_volume(ES_MODULE_ADC, 0, 0);      // 0db
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0x09); //Power on ADC, Enable LIN&RIN, Power off MICBIAS, set int1lp to low power mode

    return res;
}

// This function sets the I2S format which can be one of
//		I2S_NORMAL
//		I2S_LEFT		Left Justified
//		I2S_RIGHT,      Right Justified
//		I2S_DSP,        dsp/pcm format
//
// and the bits per sample which must be one of
//		BIT_LENGTH_16BITS
//		BIT_LENGTH_18BITS
//		BIT_LENGTH_20BITS
//		BIT_LENGTH_24BITS
//		BIT_LENGTH_32BITS
//
// Note the above must match the ESP-IDF I2S configuration which is set separately

esp_err_t es8388_config_i2s( es_bits_length_t bits_length, es_module_t mode, es_format_t fmt )
{
    esp_err_t res = ESP_OK;
    uint8_t reg = 0;

    // Set the Format
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        printf( "Setting I2S ADC Format\n");
        res = es_read_reg(ES8388_ADCCONTROL4, &reg);
        reg = reg & 0xfc;
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL4, reg | fmt);
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        printf( "Setting I2S DAC Format\n");
        res = es_read_reg(ES8388_DACCONTROL1, &reg);
        reg = reg & 0xf9;
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL1, reg | (fmt << 1));
    }


    // Set the Sample bits length
    int bits = (int)bits_length;
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        printf( "Setting I2S ADC Bits: %d\n", bits);
        res = es_read_reg(ES8388_ADCCONTROL4, &reg);
        reg = reg & 0xe3;
        res |=  es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL4, reg | (bits << 2));
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        printf( "Setting I2S DAC Bits: %d\n", bits);
        res = es_read_reg(ES8388_DACCONTROL1, &reg);
        reg = reg & 0xc7;
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL1, reg | (bits << 3));
    }
    return res;
}


esp_err_t es8388_set_voice_mute(bool enable)
{
    esp_err_t res = ESP_OK;
    uint8_t reg = 0;
    res = es_read_reg(ES8388_DACCONTROL3, &reg);
    reg = reg & 0xFB;
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL3, reg | (((int)enable) << 2));
    return res;
}

esp_err_t es8388_start(es_module_t mode)
{
    esp_err_t res = ESP_OK;
    uint8_t prev_data = 0, data = 0;
    es_read_reg(ES8388_DACCONTROL21, &prev_data);
    if (mode == ES_MODULE_LINE) {
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL16, 0x09); // 0x00 audio on LIN1&RIN1,  0x09 LIN2&RIN2 by pass enable
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL17, 0x50); // left DAC to left mixer enable  and  LIN signal to left mixer enable 0db  : bupass enable
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL20, 0x50); // right DAC to right mixer enable  and  LIN signal to right mixer enable 0db : bupass enable
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0xC0); //enable adc
    } else {
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x80);   //enable dac
    }
    es_read_reg(ES8388_DACCONTROL21, &data);
    if (prev_data != data) {
    	printf( "Resetting State Machine\n");

        res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0xF0);   //start state machine
        // res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL1, 0x16);
        // res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL2, 0x50);
        res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0x00);   //start state machine
    }
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC || mode == ES_MODULE_LINE) {
    	printf( "Powering up ADC\n");
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0x00);   //power up adc and line in
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC || mode == ES_MODULE_LINE) {
    	printf( "Powering up DAC\n");
        res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x3c);   //power up dac and line out
        res |= es8388_set_voice_mute(false);
    }

    return res;
}


esp_err_t es8388_set_voice_volume(int volume)
{
    esp_err_t res = ESP_OK;
    if (volume < 0)
        volume = 0;
    else if (volume > 100)
        volume = 100;
    volume /= 3;
    res = es_write_reg(ES8388_ADDR, ES8388_DACCONTROL24, volume);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL25, volume);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL26, 0);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL27, 0);
    return res;
}


void es8388_config()
{
    // Input/Output Modes
    //
    //	es_dac_output_t output = DAC_OUTPUT_LOUT1 | DAC_OUTPUT_LOUT2 | DAC_OUTPUT_ROUT1 | DAC_OUTPUT_ROUT2;
    //	es_adc_input_t input = ADC_INPUT_LINPUT1_RINPUT1;
    // 	es_adc_input_t input = ADC_INPUT_LINPUT2_RINPUT2;

    //es_dac_output_t output = DAC_OUTPUT_LOUT1 | DAC_OUTPUT_LOUT2 | DAC_OUTPUT_ROUT1 | DAC_OUTPUT_ROUT2;
	es_dac_output_t output = DAC_OUTPUT_LOUT1  | DAC_OUTPUT_ROUT1;
	//es_dac_output_t output = DAC_OUTPUT_LOUT2  | DAC_OUTPUT_ROUT2;
	es_adc_input_t input = ADC_INPUT_LINPUT1_RINPUT1;

    es8388_init( output, input );

    // Modes Available
    //
    //	es_mode_t  = ES_MODULE_ADC;
    //	es_mode_t  = ES_MODULE_LINE;
    //	es_mode_t  = ES_MODULE_DAC;
    //	es_mode_t  = ES_MODULE_ADC_DAC;

    es_bits_length_t bits_length = BIT_LENGTH_16BITS;
    es_module_t module = ES_MODULE_DAC;
    es_format_t fmt = I2S_NORMAL;

    es8388_config_i2s( bits_length, ES_MODULE_ADC_DAC, fmt );
    es8388_set_voice_volume( VOL_DEFAULT );
    es8388_start( module );

}


esp_err_t i2s_mclk_gpio_select(i2s_port_t i2s_num, gpio_num_t gpio_num)
{

	// Ignore whatever is sent in and fix to Pin 3

//	gpio_num = GPIO_NUM_3;

    if (i2s_num >= I2S_NUM_MAX) {
        ESP_LOGE(ES_TAG, "Does not support i2s number(%d)", i2s_num);
        return ESP_ERR_INVALID_ARG;
    }
    if (gpio_num != GPIO_NUM_0 && gpio_num != GPIO_NUM_1 && gpio_num != GPIO_NUM_3) {
        ESP_LOGE(ES_TAG, "Only support GPIO0/GPIO1/GPIO3, gpio_num:%d", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGI(ES_TAG, "I2S%d, MCLK output by GPIO%d", i2s_num, gpio_num);
    if (i2s_num == I2S_NUM_0) {
        if (gpio_num == GPIO_NUM_0) {
            PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
            WRITE_PERI_REG(PIN_CTRL, 0xFFF0);
        } else if (gpio_num == GPIO_NUM_1) {
            PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD_CLK_OUT3);
            WRITE_PERI_REG(PIN_CTRL, 0xF0F0);
        } else {
            PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_U0RXD_CLK_OUT2);
            WRITE_PERI_REG(PIN_CTRL, 0xFF00);
        }
    } else if (i2s_num == I2S_NUM_1) {
        if (gpio_num == GPIO_NUM_0) {
            PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
            WRITE_PERI_REG(PIN_CTRL, 0xFFFF);
        } else if (gpio_num == GPIO_NUM_1) {
            PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD_CLK_OUT3);
            WRITE_PERI_REG(PIN_CTRL, 0xF0FF);
        } else {
            PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_U0RXD_CLK_OUT2);
            WRITE_PERI_REG(PIN_CTRL, 0xFF0F);
        }
    }
    return ESP_OK;
}



void i2s_init()
{

	i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_MSB,
        .dma_buf_count = 3,
        .dma_buf_len = 300,
        .use_apll = true,
	    .tx_desc_auto_clear = true,
	    .fixed_mclk = 0,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK_IO,
        .ws_io_num = I2S_WS_IO,
        .data_out_num = I2S_DO_IO,
        .data_in_num = I2S_DI_IO                                               //Not used
    };
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM, &pin_config);

//    i2s_mclk_gpio_select(I2S_NUM, (gpio_num_t)GPIO_NUM_3 );
    i2s_mclk_gpio_select(I2S_NUM, (gpio_num_t)IS2_MCLK_PIN );

    i2s_set_clk(I2S_NUM, SAMPLE_RATE, 16, 2);

}


static void setup_sine_waves16()
{
	double sin_float;

    size_t i2s_bytes_write = 0;

    //printf("\r\nFree mem=%d, written data=%d\n", esp_get_free_heap_size(), BUF_SAMPLES*2 );

    for( int pos = 0; pos < BUF_SAMPLES; pos += 2 )
    {
        sin_float = 10000 * sin( pos/2 * 2 * PI / SAMPLE_PER_CYCLE);

        int lval = sin_float;
        int rval = sin_float;

        txBuf[pos] = lval&0xFFFF;
        txBuf[pos+1] = rval&0xFFFF;

//        printf( "%d  %04x:%04x\n", lval, txBuf[pos],txBuf[pos+1] );

    }
}

void app_main(void)
{
	i2c_master_init();
	es8388_config();

	es8388_read_all();


	i2s_init();

    size_t i2s_bytes_write = 0;

    setup_sine_waves16();

    while (1) {

    	setup_sine_waves16();

    	//i2s_write(I2S_NUM, txBuf, BUF_SAMPLES*2, &i2s_bytes_write, -1);
    	i2s_write(I2S_NUM, txBuf, BUF_SAMPLES*2, &i2s_bytes_write, -1);
    	//printf( "Bytes: %d\n", i2s_bytes_write );
    	//vTaskDelay(10/portTICK_RATE_MS);

    }
}

