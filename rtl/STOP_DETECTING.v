module STOP_DETECTING(
input wire clk_50m,
input wire rst,
input wire distance_vld_go,
input wire gas_alarm,
input wire [8:0] key_value,
input wire [8:0] key_sensor,
input wire [8:0] key_cmd,
output reg new_stop_event,
output reg [7:0] curr_stop,
input wire [3:0] car_state,
input wire bt_lock,
input wire sensor_lock,
input wire distance_vld_back
);

  //----Code starts here: integrated by Robei-----
  parameter STOP_NONE=8'd0;
  parameter STOP_GAS=8'd1;
  parameter STOP_OBSTACLE=8'd2;
  parameter STOP_KEY=8'd3;
  reg [7:0]last_curr_stop;
  reg prev_stop;
  always @ ( posedge clk_50m or posedge rst )
  begin 
  	if(rst)
      curr_stop<=STOP_NONE;
      else begin
      		if(gas_alarm)
              curr_stop<=STOP_GAS;
              else begin
              		if(distance_vld_go||distance_vld_back)
                      curr_stop<=STOP_OBSTACLE;
                      else begin
                              		if(key_value==9'b111_101_111&&(~sensor_lock)&&(~bt_lock)||(key_cmd==9'b111_101_111&&bt_lock))
                                      curr_stop<=STOP_KEY;
                                      end
                              end
                      end
              end
  always @ ( posedge clk_50m or posedge rst)
  begin 
  	if(rst)begin
      new_stop_event<=0;
      prev_stop<=1;
      last_curr_stop<=0;
      end
      else begin
      prev_stop<=(car_state==4'b0000);
      last_curr_stop<=curr_stop;
      new_stop_event<=(car_state==4'b0000 && (prev_stop==0)||car_state==4'b0000 && (curr_stop!=last_curr_stop));
      end   
  end
endmodule    //STOP_DETECTING
