module DATA_PACK_SEND(
input wire clk_50m,
input wire rst,
input wire [23:0] weight_data,
input wire [3:0] car_state,
input wire gas_alarm,
output reg tx_data_vld,
output reg [7:0] tx_data,
input wire en_500,
input wire [7:0] curr_stop,
input wire new_stop_event,
input wire heavy_load_state,
input wire [15:0] actual_speed_left,
input wire [15:0] actual_speed_right,
input wire [15:0] target_speed_r,
input wire [15:0] target_speed_l,
input wire [15:0] distance_mm2,
input wire [15:0] distance_mm1,
input wire [3:0] error_mpu,
input wire mpu_dir,
input wire mpu_dir_z,
input wire [3:0] error_mpu_z,
input wire sensor_lost,
input wire [7:0] sensor_fault,
input wire [3:0] track_confidence,
input wire [1:0] control_mode,
input wire [1:0] load_event_state,
input wire load_event_toggle,
input wire [23:0] load_change_amount,
input wire settle_timeout_fault,
input wire overload_state
);

  //----Code starts here: integrated by Robei-----
  //本来我是把FAULT_LOGGER模块单独写的，但是超过了20个模块的限额，所以只能把FAULT_LOGGER放这里写的
  reg [2:0]fault_0;
  reg [2:0]fault_1;
  reg [2:0]fault_2;
  reg [2:0]fault_3;
  reg [2:0]fault_4;
  always @ (posedge clk_50m or posedge rst  )
    begin 
     if(rst)begin
        fault_4<=0;
        fault_3<=0;
        fault_2<=0;
        fault_1<=0;
        fault_0<=0;
        end
        else begin
          if(new_stop_event)begin
                fault_4<=fault_3;
                fault_3<=fault_2;
                fault_2<=fault_1;
                fault_1<=fault_0;
                fault_0<=curr_stop;
                end
        end
    end
  reg [7:0]step;
  always @ ( posedge clk_50m or posedge rst )
  begin 
  	if(rst)begin
      tx_data<=0;
      tx_data_vld<=0;
      step<=0;
      end
      else if(en_500)
      begin
      step<=(step==23)?0:step+1;
      tx_data_vld<=1;
      case(step)
      0:tx_data<=8'hAA;
      1:tx_data<={4'd0,car_state};
      2:tx_data<={6'd0,overload_state,gas_alarm};
      3:tx_data<=weight_data[23:16];
      4:tx_data<=weight_data[15:8];
      5:tx_data<=weight_data[7:0];
      6:tx_data<=fault_0;
      7:tx_data<=fault_1;
      8:tx_data<=fault_2;
      9:tx_data<=fault_3;
      10:tx_data<=fault_4;
      11:tx_data<=distance_mm1[7:0];
       12:tx_data<=distance_mm1[15:8];
      13:tx_data<=distance_mm2[7:0];
      14:tx_data<=distance_mm2[15:8];
      15:tx_data<=actual_speed_left[7:0];
          16:tx_data<=actual_speed_left[15:8];
              17:tx_data<=actual_speed_right[7:0];
                  18:tx_data<=actual_speed_right[15:8];
                      19:tx_data<=target_speed_l[7:0];
                      20:tx_data<=target_speed_l[15:8];
                      21:tx_data<=target_speed_r[7:0];
                      22:tx_data<=target_speed_r[15:8];
                  
      23:tx_data<=8'h55;
      default:begin
      tx_data<=0;
      step<=0;
      tx_data_vld<=0;
      end
      endcase
      end
      else tx_data_vld<=0;
  end
endmodule    //DATA_PACK_SEND
