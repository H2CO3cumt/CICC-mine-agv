module CORE_CONTROL_TOP(
input wire clk_50m,
input wire en_1k,
input wire rst,
output wire pwm_left,
input wire [8:0] key_value,
output wire pwm_right,
output wire [3:0] car_state,
input wire load_state,
input wire rx,
input wire [8:0] key_sensor,
input wire distance_vld,
input wire force_stop,
input wire [23:0] weight_data,
input wire gas_alarm,
output wire tx,
input wire en_500,
output wire IN1,
output wire IN2,
output wire IN3,
output wire IN4,
output wire p23,
input wire mpu_dir,
input wire [3:0] error_mpu,
input wire enc_left_a,
input wire enc_left_b,
input wire enc_right_a,
input wire enc_right_b,
input wire [3:0] error_mag,
input wire turn_dir,
input wire sensor_lost,
input wire [15:0] distance_mm2,
input wire [15:0] distance_mm1,
input wire distance_vld_go,
input wire distance_vld_back,
input wire overload_state,
input wire heavy_load_state,
input wire [23:0] load_change_amount,
input wire load_event_toggle
);
  wire CAR_STATE1_en;
  wire [8:0] BT_CTRL3_key_cmd;
  wire CAR_STATE1_sensor_lock;
  wire CAR_STATE1_bt_lock;
  wire [3:0] CAR_STATE1_car_state1;
  wire [15:0] PWM2_actual_speed_right;
  wire [15:0] PWM2_actual_speed_left;
  wire [15:0] PWM2_target_speed_r;
  wire [15:0] PWM2_target_speed_l;
  assign car_state = CAR_STATE1_car_state1;

  //----Code starts here: integrated by Robei-----
  
//---Module instantiation---
  CAR_STATE CAR_STATE1(
    .key_value(key_value),
    .en(CAR_STATE1_en),
    .key_cmd(BT_CTRL3_key_cmd),
    .en_1k(en_1k),
    .clk_50m(clk_50m),
    .rst(rst),
    .key_sensor(key_sensor),
    .force_stop(force_stop),
    .bt_lock(CAR_STATE1_bt_lock),
    .sensor_lock(CAR_STATE1_sensor_lock),
    .car_state1(CAR_STATE1_car_state1),
    .distance_vld_go(distance_vld_go),
    .distance_vld_back(distance_vld_back),
    .overload_state(overload_state)
);

  PWM PWM2(
    .car_state(CAR_STATE1_car_state1),
    .clk_50m(clk_50m),
    .pwm_left(pwm_left),
    .en(CAR_STATE1_en),
    .pwm_right(pwm_right),
    .rst(rst),
    .en_1k(en_1k),
    .IN1(IN1),
    .IN2(IN2),
    .IN3(IN3),
    .IN4(IN4),
    .error_mag(error_mag),
    .turn_dir(turn_dir),
    .sensor_lock(CAR_STATE1_sensor_lock),
    .enc_left_a(enc_left_a),
    .enc_left_b(enc_left_b),
    .enc_right_a(enc_right_a),
    .enc_right_b(enc_right_b),
    .mpu_dir(mpu_dir),
    .error_mpu(error_mpu),
    .sensor_lost(sensor_lost),
    .force_stop(force_stop),
    .distance_vld(distance_vld),
    .target_speed_l(PWM2_target_speed_l),
    .target_speed_r(PWM2_target_speed_r),
    .actual_speed_left(PWM2_actual_speed_left),
    .actual_speed_right(PWM2_actual_speed_right),
    .heavy_load_state(heavy_load_state)
);

  BT_CTRL BT_CTRL3(
    .clk_50m(clk_50m),
    .rx(rx),
    .key_cmd(BT_CTRL3_key_cmd),
    .rst(rst),
    .rx_data(),
    .rx_data_vld()
);

  BT_RECEIVING BT_RECEIVING4(
    .clk_50m(clk_50m),
    .rst(rst),
    .car_state(CAR_STATE1_car_state1),
    .gas_alarm(gas_alarm),
    .weight_data(weight_data),
    .key_sensor(key_sensor),
    .key_value(key_value),
    .key_cmd(BT_CTRL3_key_cmd),
    .distance_vld(distance_vld),
    .tx(tx),
    .en_500(en_500),
    .bt_lock(CAR_STATE1_bt_lock),
    .sensor_lock(CAR_STATE1_sensor_lock),
    .actual_speed_left(PWM2_actual_speed_left),
    .actual_speed_right(PWM2_actual_speed_right),
    .target_speed_l(PWM2_target_speed_l),
    .target_speed_r(PWM2_target_speed_r),
    .distance_mm2(distance_mm2),
    .distance_mm1(distance_mm1),
    .load_event_state(),
    .overload_state(),
    .settle_time_fault()
);

endmodule    //CORE_CONTROL_TOP
