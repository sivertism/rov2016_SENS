/* CANBUS filter match indexes. (FMI) */
volatile extern uint8_t fmi_topside_xbox_ctrl;
volatile extern uint8_t fmi_topside_xbox_axes;
volatile extern uint8_t fmi_vesc_current_9;
volatile extern uint8_t fmi_vesc_rpm_9;
volatile extern uint8_t fmi_vesc_mosfet_temperature_6;

/* Global software flags */
volatile extern uint8_t flag_systick_update_attitude;
volatile extern uint8_t flag_systick_update_heading;
volatile extern uint8_t flag_systick_transmit_thrust;
volatile extern uint8_t flag_systick_update_depth;
volatile extern uint8_t flag_systick_update_ms5803_temp;

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
extern uint8_t magnetometer_data[6];
