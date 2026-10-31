module BT_RECEIVING(
input wire clk_50m,
input wire rst,
input wire en_500,
input wire [3:0] car_state,
input wire gas_alarm,
input wire [23:0] weight_data,
input wire [8:0] key_sensor,
input wire [8:0] key_value,
input wire [8:0] key_cmd,
input wire distance_vld,
input wire load_event_state,
output wire tx,
input wire bt_lock,
input wire sensor_lock,
input wire [15:0] actual_speed_left,
input wire [15:0] actual_speed_right,
input wire [15:0] target_speed_l,
input wire [15:0] target_speed_r,
input wire [15:0] distance_mm2,
input wire [15:0] distance_mm1,
input wire overload_state,
input wire settle_time_fault
);
  wire DATA_PACK_SEND1_tx_data_vld;
  wire [7:0] DATA_PACK_SEND1_tx_data;
  wire [7:0] STOP_DETECTING4_curr_stop;
  wire STOP_DETECTING4_new_stop_event;

  //----Code starts here: integrated by Robei-----
  
//---Module instantiation---
  DATA_PACK_SEND DATA_PACK_SEND1(
    .clk_50m(clk_50m),
    .rst(rst),
    .weight_data(weight_data),
    .car_state(car_state),
    .gas_alarm(gas_alarm),
    .tx_data_vld(DATA_PACK_SEND1_tx_data_vld),
    .tx_data(DATA_PACK_SEND1_tx_data),
    .en_500(en_500),
    .curr_stop(STOP_DETECTING4_curr_stop),
    .new_stop_event(STOP_DETECTING4_new_stop_event),
    .load_state(load_event_state),
    .actual_speed_left(actual_speed_left),
    .actual_speed_right(actual_speed_right),
    .target_speed_l(target_speed_l),
    .target_speed_r(target_speed_r),
    .distance_mm2(distance_mm2),
    .distance_mm1(distance_mm1)
);

  UART_TX UART_TX3(
    .clk_50m(clk_50m),
    .tx_data_vld(DATA_PACK_SEND1_tx_data_vld),
    .tx_data(DATA_PACK_SEND1_tx_data),
    .ready(),
    .tx(tx),
    .rst(rst)
);

  STOP_DETECTING STOP_DETECTING4(
    .clk_50m(clk_50m),
    .rst(rst),
    .gas_alarm(gas_alarm),
    .key_value(key_value),
    .key_sensor(key_sensor),
    .key_cmd(key_cmd),
    .new_stop_event(STOP_DETECTING4_new_stop_event),
    .curr_stop(STOP_DETECTING4_curr_stop),
    .car_state(car_state),
    .bt_lock(bt_lock),
    .sensor_lock(sensor_lock),
    .distance_vld_go(),
    .distance_vld_back()
);

endmodule    //BT_RECEIVING
