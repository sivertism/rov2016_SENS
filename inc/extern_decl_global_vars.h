/* CANBUS filter match indexes. (FMI) */
volatile extern uint8_t fmi_topside_xbox_ctrl;
volatile extern uint8_t fmi_topside_xbox_axes;
volatile extern uint8_t fmi_topside_sens_ctrl;
volatile extern uint8_t fmi_topside_reg_param1;
volatile extern uint8_t fmi_topside_reg_param2;
volatile extern uint8_t fmi_auto_thrust;

/* Global software flags */
volatile extern uint8_t flag_systick_update_attitude;
volatile extern uint8_t flag_systick_update_heading;
volatile extern uint8_t flag_systick_transmit_thrust;
volatile extern uint8_t flag_systick_update_depth;
volatile extern uint8_t flag_systick_update_ms5803_temp;
volatile extern uint8_t flag_systick_calibrate_gyro;
volatile extern uint8_t flag_systick_zero_pressure;
volatile extern uint8_t flag_systick_update_temp;
volatile extern uint8_t flag_systick_auto;
volatile extern uint8_t flag_systick_update_leak;


/* Parameter for regulator */
volatile extern int32_t Kp_t;
volatile extern int32_t Ti_t;
volatile extern int32_t Td_t;
volatile extern int32_t Kp_r;
volatile extern int32_t Ti_r;
volatile extern int32_t Td_r;
volatile extern uint16_t Ts;

/* Status LED's */
extern const uint8_t MAIN_LOOP_LED;
extern const uint8_t SYSTICK_LED;
extern const uint8_t CAN_RX_LED;
extern const uint8_t CAN_TX_LED;
extern const uint8_t UART_RX_LED;
extern const uint8_t UART_TX_LED;
extern const uint8_t STATUS_LED6;
extern const uint8_t STATUS_LED7;

extern uint8_t accelerometer_data[6];
extern int32_t depth;
extern int32_t* attitude;
