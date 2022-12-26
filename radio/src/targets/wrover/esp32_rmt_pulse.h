
#ifndef _ESP32_RMT_PULSE_H_
#define _ESP32_RMT_PULSE_H_

typedef struct _rmt_ctx_ rmt_ctx_t;

// encode data to RMT format
// return value indicating count of data to be TX
typedef size_t (*rmt_tx_encode_cb_t)(rmt_ctx_t *ctx);
// decode RMT format to data
typedef void (*rmt_rx_decode_cb_t)(rmt_ctx_t *ctx, uint32_t *rxdata, size_t rxdata_len);

struct _rmt_ctx_ {
    rmt_obj_t *rmt;
    bool exit;
    size_t pulse_in_frame;
    rmt_data_t *data;
    float tick_in_ns;
    TaskHandle_t task_id;
    StaticTask_t task_struct;
    StackType_t rmt_tx_stack[1024*2];
    rmt_tx_encode_cb_t encoder;
    rmt_rx_decode_cb_t decoder;
};

rmt_ctx_t *esp32_rmt_tx_init(int pin, rmt_reserve_memsize_t memsize, float tick_in_ns, rmt_tx_encode_cb_t enc_fn, size_t pulse_in_frame);
rmt_ctx_t *esp32_rmt_rx_init(int pin, rmt_reserve_memsize_t memsize, float tick_in_ns, rmt_rx_decode_cb_t dec_fn, size_t pulse_in_frame, size_t idle_threshold_in_ns, size_t min_pulse_in_ns = 0);
void esp32_rmt_stop(rmt_ctx_t *ctx);

// return number of channel decoded
int rmt_ppm_decode_cb(rmt_ctx_t *ctx, uint32_t *rxdata, size_t rxdata_len, int16_t *ppm_decode_buf);
void rmt_ppm_encode_cb(rmt_ctx_t *ctx, uint16_t *ppm, size_t len);

#define RMT_PPM_OUT_TICK_NS 500  // Upper layer uses 0.5us
#define RMT_PPM_IN_TICK_NS 1000
#define RMT_PPM_IDLE_THRESHOLD_NS 4000000 // 4ms

#endif