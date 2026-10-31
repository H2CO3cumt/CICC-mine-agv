module TOP(
input wire clk_50m,
input wire [8:0] key_in,
output wire [1:0] right,
output wire [1:0] go,
output wire [1:0] back,
output wire [7:0] seg,
output wire [5:0] sel,
output wire buzz_out,
output wire pwm_left,
output wire dir_left,
input wire rx,
output wire [1:0] left,
output wire U20,
output wire tx,
input wire [7:0] direction_in,
output wire pwm_right,
input wire enc_left_a,
input wire enc_left_b,
input wire enc_right_a,
input wire enc_right_b,
input wire dout,
input wire echo_1,
input wire do,
output wire sck,
output wire trig_1,
output wire IN1,
output wire IN2,
output wire IN3,
output wire IN4,
inout wire sda,
output wire P19,
output wire scl,
input wire echo_2,
output wire trig_2
);
  wire [8:0] UI_CMD_TOP3_key_value;
  wire [3:0] CORE_CONTROL_TOP2_car_state;
  wire CLK_DIV_TOP1_en_1;
  wire CLK_DIV_TOP1_en_1k;
  wire CLK_DIV_TOP1_en_10k;
  wire CLK_DIV_TOP1_en_500;
  wire CLK_DIV_TOP1_en_2k;
  wire CLK_DIV_TOP1_rst;
  wire SENSOR_TOP4_distance_vld;
  wire SENSOR_TOP4_force_stop;
  wire SENSOR_TOP4_gas_alarm;
  wire [23:0] SENSOR_TOP4_weight_data;
  wire CLK_DIV_TOP1_en_100;
  wire SENSOR_TOP4_turn_dir;
  wire SENSOR_TOP4_mpu_dir;
  wire [3:0] SENSOR_TOP4_error_mpu;
  wire [3:0] SENSOR_TOP4_error_mag;
  wire SENSOR_TOP4_sensor_lost;
  wire [15:0] SENSOR_TOP4_distance_mm1;
  wire [15:0] SENSOR_TOP4_distance_mm2;
  wire SENSOR_TOP4_distance_vld_back;
  wire SENSOR_TOP4_distance_vld_go;
  wire SENSOR_TOP4_overload_state;
  wire [23:0] SENSOR_TOP4_load_change_amount;
  wire SENSOR_TOP4_load_event_toggle;
  wire SENSOR_TOP4_heavy_load_state;

  //----Code starts here: integrated by Robei-----
  assign U20=1;
  assign P19=1;
//---Module instantiation---
  CLK_DIV_TOP CLK_DIV_TOP1(
    .clk_50m(clk_50m),
    .en_1(CLK_DIV_TOP1_en_1),
    .en_1k(CLK_DIV_TOP1_en_1k),
    .en_10k(CLK_DIV_TOP1_en_10k),
    .en_1m(),
    .en_500(CLK_DIV_TOP1_en_500),
    .en_2k(CLK_DIV_TOP1_en_2k),
    .rst(CLK_DIV_TOP1_rst),
    .key_value(UI_CMD_TOP3_key_value),
    .en_10(),
    .en_100(CLK_DIV_TOP1_en_100)
);

  CORE_CONTROL_TOP CORE_CONTROL_TOP2(
    .clk_50m(clk_50m),
    .pwm_left(pwm_left),
    .key_value(UI_CMD_TOP3_key_value),
    .pwm_right(pwm_right),
    .car_state(CORE_CONTROL_TOP2_car_state),
    .load_state(),
    .en_1k(CLK_DIV_TOP1_en_1k),
    .rx(rx),
    .rst(CLK_DIV_TOP1_rst),
    .key_sensor(),
    .distance_vld(SENSOR_TOP4_distance_vld),
    .force_stop(SENSOR_TOP4_force_stop),
    .weight_data(SENSOR_TOP4_weight_data),
    .gas_alarm(SENSOR_TOP4_gas_alarm),
    .tx(tx),
    .en_500(CLK_DIV_TOP1_en_500),
    .IN1(IN1),
    .IN2(IN2),
    .IN3(IN3),
    .IN4(IN4),
    .p23(),
    .mpu_dir(SENSOR_TOP4_mpu_dir),
    .error_mpu(SENSOR_TOP4_error_mpu),
    .enc_left_a(enc_left_a),
    .enc_left_b(enc_left_b),
    .enc_right_a(enc_right_a),
    .enc_right_b(enc_right_b),
    .error_mag(SENSOR_TOP4_error_mag),
    .turn_dir(SENSOR_TOP4_turn_dir),
    .sensor_lost(SENSOR_TOP4_sensor_lost),
    .distance_mm2(SENSOR_TOP4_distance_mm2),
    .distance_mm1(SENSOR_TOP4_distance_mm1),
    .distance_vld_go(SENSOR_TOP4_distance_vld_go),
    .distance_vld_back(SENSOR_TOP4_distance_vld_back),
    .overload_state(SENSOR_TOP4_overload_state),
    .heavy_load_state(SENSOR_TOP4_heavy_load_state),
    .load_change_amount(SENSOR_TOP4_load_change_amount),
    .load_event_toggle(SENSOR_TOP4_load_event_toggle)
);

  UI_CMD_TOP UI_CMD_TOP3(
    .key_value(UI_CMD_TOP3_key_value),
    .sel(sel),
    .seg(seg),
    .car_state(CORE_CONTROL_TOP2_car_state),
    .buzz_out(buzz_out),
    .key_in(key_in),
    .en_10k(CLK_DIV_TOP1_en_10k),
    .en_1(CLK_DIV_TOP1_en_1),
    .en_2k(CLK_DIV_TOP1_en_2k),
    .en_500(CLK_DIV_TOP1_en_500),
    .clk_50m(clk_50m),
    .rst(CLK_DIV_TOP1_rst),
    .gas_alarm(SENSOR_TOP4_gas_alarm),
    .back(back),
    .go(go),
    .right(right),
    .left(left)
);

  SENSOR_TOP SENSOR_TOP4(
    .clk_50m(clk_50m),
    .rst(CLK_DIV_TOP1_rst),
    .direction_in(direction_in),
    .dout(dout),
    .do(do),
    .distance_vld(SENSOR_TOP4_distance_vld),
    .sck(sck),
    .gas_alarm(SENSOR_TOP4_gas_alarm),
    .force_stop(SENSOR_TOP4_force_stop),
    .en_10k(CLK_DIV_TOP1_en_10k),
    .weight_data(SENSOR_TOP4_weight_data),
    .weight_data_vld(),
    .sda(sda),
    .en_100(CLK_DIV_TOP1_en_100),
    .scl(scl),
    .mpu_dir(SENSOR_TOP4_mpu_dir),
    .error_mpu(SENSOR_TOP4_error_mpu),
    .turn_dir(SENSOR_TOP4_turn_dir),
    .sensor_lost(SENSOR_TOP4_sensor_lost),
    .error_mag(SENSOR_TOP4_error_mag),
    .calib_done(),
    .ack_err_code(),
    .car_state(CORE_CONTROL_TOP2_car_state),
    .distance_mm2(SENSOR_TOP4_distance_mm2),
    .distance_mm1(SENSOR_TOP4_distance_mm1),
    .echo_1(echo_1),
    .trig_1(trig_1),
    .distance_vld_go(SENSOR_TOP4_distance_vld_go),
    .distance_vld_back(SENSOR_TOP4_distance_vld_back),
    .echo_2(echo_2),
    .trig_2(trig_2),
    .heavy_load_state(SENSOR_TOP4_heavy_load_state),
    .overload_state(SENSOR_TOP4_overload_state),
    .settle_time_fault(),
    .load_event_state(),
    .load_event_toggle(SENSOR_TOP4_load_event_toggle),
    .load_change_amount(SENSOR_TOP4_load_change_amount)
);

endmodule    //TOP
