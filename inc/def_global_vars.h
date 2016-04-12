/* CANBUS filter match indexes. (FMI) */
volatile uint8_t fmi_topside_xbox_ctrl = 0;
volatile uint8_t fmi_topside_xbox_axes = 0;
volatile uint8_t fmi_vesc_current_9 = 0;
volatile uint8_t fmi_vesc_rpm_9 = 0;
volatile uint8_t fmi_vesc_mosfet_temperature_6 = 0;

/* Global software flags */
volatile uint8_t flag_systick_update_heading = 0;
volatile uint8_t flag_systick_update_attitude = 0;
volatile uint8_t flag_systick_transmit_thrust = 0;
volatile uint8_t flag_systick_update_depth = 0;
volatile uint8_t flag_systick_update_ms5803_temp = 0;

/* Choice of axis in accelerometer_data */
const uint8_t ACC_AXIS_X				= 0;
const uint8_t ACC_AXIS_Y				= 1;
const uint8_t ACC_AXIS_Z				= 2;

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
