/* CANBUS filter match indexes. (FMI) */
volatile uint8_t fmi_topside_xbox_ctrl = 0;
volatile uint8_t fmi_topside_xbox_axes = 0;
volatile uint8_t fmi_topside_sens_ctrl = 0;
volatile uint8_t fmi_topside_reg_param1 = 0;
volatile uint8_t fmi_topside_reg_param2 = 0;
volatile uint8_t fmi_auto_thrust = 0;

/* Global software flags */
volatile uint8_t flag_systick_update_heading = 0;
volatile uint8_t flag_systick_update_attitude = 0;
volatile uint8_t flag_systick_transmit_thrust = 0;
volatile uint8_t flag_systick_update_depth = 0;
volatile uint8_t flag_systick_update_ms5803_temp = 0;
volatile uint8_t flag_systick_calibrate_gyro = 0;
volatile uint8_t flag_systick_update_temp = 0;
volatile uint8_t flag_systick_zero_pressure = 0;
volatile uint8_t flag_systick_auto = 0;
volatile uint8_t flag_systick_update_leak = 0;

/* Regulator parameters*/
volatile int32_t Kp_t = 0;
volatile int32_t Ti_t = 0;
volatile int32_t Td_t = 0;
volatile int32_t Kp_r = 0;
volatile int32_t Ti_r = 0;
volatile int32_t Td_r = 0;
volatile uint16_t Ts  = 100;

/* Status LED's */
const uint8_t MAIN_LOOP_LED				= (1u << 0);
const uint8_t SYSTICK_LED				= (1u << 1);
const uint8_t CAN_RX_LED				= (1u << 2);
const uint8_t CAN_TX_LED				= (1u << 3);
const uint8_t UART_RX_LED				= (1u << 4);
const uint8_t UART_TX_LED				= (1u << 5);
const uint8_t STATUS_LED6				= (1u << 6);
const uint8_t STATUS_LED7				= (1u << 7);

uint8_t accelerometer_data[6] = {0};
int32_t depth = 0;
int32_t* attitude = {0};
