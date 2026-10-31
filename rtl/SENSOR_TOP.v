module SENSOR_TOP(
input wire clk_50m,
input wire rst,
input wire echo_1,
input wire [7:0] direction_in,
input wire dout,
input wire do,
output wire trig_1,
output wire distance_vld,
output wire heavy_load_state,
output wire sck,
output wire gas_alarm,
output wire force_stop,
input wire en_10k,
output wire [23:0] weight_data,
output wire weight_data_vld,
inout wire sda,
input wire en_100,
output wire scl,
output wire mpu_dir,
output wire [3:0] error_mpu,
output wire turn_dir,
output wire sensor_lost,
output wire [3:0] error_mag,
output wire calib_done,
output wire [3:0] ack_err_code,
input wire [3:0] car_state,
output wire [15:0] distance_mm2,
output wire [15:0] distance_mm1,
output wire distance_vld_go,
output wire distance_vld_back,
input wire echo_2,
output wire trig_2,
output wire overload_state,
output wire settle_time_fault,
output wire [1:0] load_event_state,
output wire load_event_toggle,
output wire [23:0] load_change_amount
);

  //----Code starts here: integrated by Robei-----
  //defparam SENSOR_TOP0.GAS_SENSOR0.
//---Module instantiation---
  WEIGHT_SENSOR WEIGHT_SENSOR2(
    .clk_50m(clk_50m),
    .rst(rst),
    .dout(dout),
    .sck(sck),
    .data(weight_data),
    .data_vld(weight_data_vld),
    .heavy_load_state(heavy_load_state),
    .load_event_state(load_event_state),
    .load_event_toggle(load_event_toggle),
    .load_change_amount(load_change_amount),
    .settle_timeout_fault(settle_time_fault),
    .car_state(car_state),
    .overload_state(overload_state)
);

  TRACKING_SENSOR TRACKING_SENSOR3(
    .clk_50m(clk_50m),
    .rst(rst),
    .direction_in(direction_in),
    .en_10k(en_10k),
    .error_mag(error_mag),
    .sensor_lost(sensor_lost),
    .turn_dir(turn_dir)
);

  GAS_SENSOR GAS_SENSOR4(
    .clk_50m(clk_50m),
    .rst(rst),
    .do(do),
    .gas_alarm(gas_alarm),
    .force_stop(force_stop)
);

  ULTRASONIC_SENSOR_cg ULTRASONIC_SENSOR_cg4(
    .clk_50m(clk_50m),
    .rst(rst),
    .echo_1(echo_1),
    .distance_vld_go(distance_vld_go),
    .trig_1(trig_1),
    .echo_2(echo_2),
    .trig_2(trig_2),
    .car_state(car_state),
    .distance_vld_back(distance_vld_back),
    .distance_1(distance_mm1),
    .distance_2(distance_mm2)
);

  MPU_IIC_SENSOR MPU_IIC_SENSOR5(
    .clk_50m(clk_50m),
    .rst(rst),
    .sda(sda),
    .scl(scl),
    .raw_data(),
    .mpu_dir(mpu_dir),
    .en_100(en_100),
    .error_mpu(error_mpu),
    .ack_err_code(ack_err_code),
    .car_state(car_state)
);

endmodule    //SENSOR_TOP
