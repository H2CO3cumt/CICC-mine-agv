module ULTRASONIC_SENSOR_cg(
input wire clk_50m,
input wire rst,
input wire echo_1,
output reg distance_vld_go,
output reg trig_1,
input wire echo_2,
output reg trig_2,
input wire [3:0] car_state,
output reg distance_vld_back,
output wire [15:0] distance_1,
output wire [15:0] distance_2
);

  //----Code starts here: integrated by Robei-----
  parameter OVER_TIME=2_500_000;
  parameter WAIT_TIME=3_000_000;
  parameter TRIG_TIME=750;
  parameter IDLE=4'b0000;
  parameter SEND_TRIG=4'b0001;
  parameter WAIT_ECHO=4'b0010;
  parameter MEASURE=4'b0100;
  parameter CALC=4'b1000;
  parameter MAX_MEASURE_CNT=1_500_000;
  parameter OBSTACLE_ON=800;
  parameter OBSTACLE_OFF=850;//双阈
  wire flag_go=(car_state==4'b0001||car_state==4'b0100||car_state==4'b1000);
  wire flag_back=(car_state==4'b0010);
  wire flag_idle=(car_state==4'b0000);
  reg [15:0]distance_mm1;
  reg [15:0]distance_mm2;
  reg echo1_1;
  reg echo1_2;
  reg [23:0]cnt1;
  reg [3:0]cstate1;
  reg [3:0]nstate1;
  //reg [15:0]distance_mm1;
  reg echo2_1;
  reg echo2_2;
  reg [23:0]cnt2;
  reg [3:0]cstate2;
  reg [3:0]nstate2;
  //reg [15:0]distance_mm2;
  reg [1:0]step;
  always @ (posedge clk_50m or posedge rst )
  begin
      if(rst) begin
          cstate1<=IDLE;
          cstate2<=IDLE;
          end
      else begin
      		if(group_pulse)begin
                  cstate1<=IDLE;
      cstate2<=IDLE;
              end
      else begin    cstate1<=nstate1;
          cstate2<=nstate2;
          end
          end
  end
  always @ (posedge clk_50m or posedge rst )
  begin
      if(rst) begin
          			echo1_1<=0;
          echo1_2<=0;
                  echo2_1<=0;
          echo2_2<=0;
      end
      else begin
          			echo1_2<=echo1_1;
          echo1_1<=echo_1;
                  echo2_2<=echo2_1;
          echo2_1<=echo_2;
      end
  end
  always @ ( * )
  begin
  	nstate1=cstate1;
      	nstate2=cstate2;
      if(step==1&&(flag_go||flag_idle))
      case( cstate1)
          IDLE:if(cnt1==WAIT_TIME-1) nstate1=SEND_TRIG;
          else nstate1=IDLE;
          SEND_TRIG:if(cnt1==TRIG_TIME-1)nstate1=WAIT_ECHO;
          else nstate1=SEND_TRIG;
          WAIT_ECHO:if(echo1_2==1) nstate1=MEASURE;
          else begin
              if(cnt1<OVER_TIME-1) nstate1=WAIT_ECHO;
              else nstate1=IDLE;
          end
          MEASURE:if(echo1_2==1&& cnt1<MAX_MEASURE_CNT-1) nstate1=MEASURE;
          else nstate1=CALC;
          CALC: begin nstate1=IDLE;
          end
          default:  begin nstate1=IDLE;
          end
      endcase
                  if(step==2&&(flag_back||flag_idle))
      case( cstate2)
          IDLE:if(cnt2==WAIT_TIME-1) nstate2=SEND_TRIG;
          else nstate2=IDLE;
          SEND_TRIG:if(cnt2==TRIG_TIME-1)nstate2=WAIT_ECHO;
          else nstate2=SEND_TRIG;
          WAIT_ECHO:if(echo2_2==1) nstate2=MEASURE;
          else begin
              if(cnt2<OVER_TIME-1) nstate2=WAIT_ECHO;
              else nstate2=IDLE;
          end
          MEASURE:if(echo2_2==1&& cnt2<MAX_MEASURE_CNT-1) nstate2=MEASURE;
          else nstate2=CALC;
          CALC: begin nstate2=IDLE;
          end
          default:  begin nstate2=IDLE;
          end
      endcase
                  
  end
  reg [2:0]group_now;
  reg [2:0]group_last;
  always @ (posedge clk_50m or posedge rst  )
  begin 
  	if(rst)begin
      group_now<=0;
      group_last<=0;
      end
      else begin
      group_now<={flag_idle,flag_back,flag_go};
      group_last<=group_now;
      end
  end
  reg [15:0]distance_mm1_1;
  reg [15:0]distance_mm1_2;
  reg [15:0]distance_mm1_3;
  reg [15:0]distance_mm2_1;
  reg [15:0]distance_mm2_2;
  reg [15:0]distance_mm2_3;
  wire group_pulse=group_now[2]^group_last[2]||group_now[1]^group_last[1]||group_now[0]^group_last[0];
  always @ (posedge clk_50m or posedge rst )
  begin
      if(rst) begin
      step<=1;
          trig_1<=0;
          cnt1<=0;
          distance_mm1<=0;
                  trig_2<=0;
          cnt2<=0;
          distance_mm2<=0;
          distance_mm1_1<=0;
  distance_mm1_2<=0;
  distance_mm1_3<=0;
  distance_mm2_1<=0;
  distance_mm2_2<=0;
  distance_mm2_3<=0;
  go_flag1<=1;
  go_flag2<=1;
  go_flag3<=1;
  back_flag1<=1;
  back_flag2<=1;
  back_flag3<=1;
      end
      else 	if(flag_idle) 
      begin 
         if(group_pulse)begin
      step<=1;
      cnt1<=0;
      cnt2<=0;
      trig_1<=0;
      trig_2<=0;
      end
     else   if(step==1) begin//1	
      case(cstate1)
          IDLE:if(cnt1<WAIT_TIME-1) cnt1<=cnt1+1;
          else cnt1<=0;
          SEND_TRIG:if(cnt1<TRIG_TIME-1) begin cnt1<=cnt1+1;
              trig_1<=1;
          end
          else begin cnt1<=0;
              trig_1<=0;
          end
          WAIT_ECHO:if(echo1_2==1) cnt1<=0;
          else begin if(cnt1<OVER_TIME-1)cnt1<=cnt1+1;
              else begin cnt1<=0;
                  distance_mm1<=0;
                  step<=2;
              end
          end
          MEASURE:cnt1<=cnt1+1;
          CALC: begin distance_mm1<=((cnt1+1)*223)>>16;
                          distance_mm1_1<=distance_mm1;
                  distance_mm1_2<=distance_mm1_1;
                  distance_mm1_3<=distance_mm1_2;
              cnt1<=0; 
              step<=2; if(distance_vld_go) begin
                      go_flag1<=(distance_1>OBSTACLE_OFF && mid_value1[0]>OBSTACLE_OFF ) ;
      					go_flag2<=go_flag1;
      					go_flag3<=go_flag2;
                          end 
                          else begin
                          go_flag1<=0;
                          go_flag2<=0;
                          go_flag3<=0;
                          end
          end
          default:begin
              cnt1<=0;
              distance_mm1<=0;
                              distance_mm1_1<=0;
                  distance_mm1_2<=0;
                  distance_mm1_3<=0;
              step<=1;
              go_flag1<=0;
              go_flag2<=0;
              go_flag3<=0;
          end
      endcase
      end//1
     else  if(step==2) begin//2
      	case(cstate2)
          IDLE:if(cnt2<WAIT_TIME-1) cnt2<=cnt2+1;
          else cnt2<=0;
          SEND_TRIG:if(cnt2<TRIG_TIME-1) begin cnt2<=cnt2+1;
              trig_2<=1;
          end
          else begin cnt2<=0;
              trig_2<=0;
          end
          WAIT_ECHO:if(echo2_2==1) cnt2<=0;
          else begin if(cnt2<OVER_TIME-1)cnt2<=cnt2+1;
              else begin cnt2<=0;
                  distance_mm2<=0;
                  step<=1;
              end
          end
          MEASURE:cnt2<=cnt2+1;
          CALC: begin distance_mm2<=((cnt2+1)*223)>>16;
                          distance_mm2_1<=distance_mm2;
                  distance_mm2_2<=distance_mm2_1;
                  distance_mm2_3<=distance_mm2_2;
              cnt2<=0;
              step<=1; if(distance_vld_back) begin
                      back_flag1<=(distance_2>OBSTACLE_OFF && mid_value2[0]>OBSTACLE_OFF ) ;
      					back_flag2<=back_flag1;
      					back_flag3<=back_flag2;end
                          else begin
                          back_flag1<=0;
                          back_flag2<=0;
                          back_flag3<=0;
                          end
          end
          default:begin
              cnt2<=0;
              distance_mm2<=0;
                  distance_mm2_1<=0;
                  distance_mm2_2<=0;
                  distance_mm2_3<=0;
              step<=1;
              back_flag1<=0;
              back_flag2<=0;
              back_flag3<=0;
          end
      endcase
      end//2
      end
      else
       begin  if(flag_go) begin 
      if(group_pulse)begin
      step<=1;
      cnt1<=0;
      cnt2<=0;
      trig_1<=0;
      trig_2<=0;
      end
     else  if(step>=2)
      		step<=1;
     else   if(step==1) begin//1	
      case(cstate1)
          IDLE:if(cnt1<WAIT_TIME-1) cnt1<=cnt1+1;
          else cnt1<=0;
          SEND_TRIG:if(cnt1<TRIG_TIME-1) begin cnt1<=cnt1+1;
              trig_1<=1;
          end
          else begin cnt1<=0;
              trig_1<=0;
          end
          WAIT_ECHO:if(echo1_2==1) cnt1<=0;
          else begin if(cnt1<OVER_TIME-1)cnt1<=cnt1+1;
              else begin cnt1<=0;
                  distance_mm1<=0;
                  step<=1;
              end
          end
          MEASURE:cnt1<=cnt1+1;
          CALC: begin distance_mm1<=((cnt1+1)*223)>>16;
                                  distance_mm1_1<=distance_mm1;
                  distance_mm1_2<=distance_mm1_1;
                  distance_mm1_3<=distance_mm1_2;
              cnt1<=0;
              step<=1;		if(distance_vld_go) begin
                        go_flag1<=(distance_1>OBSTACLE_OFF && mid_value1[0]>OBSTACLE_OFF ) ;
      					go_flag2<=go_flag1;
      					go_flag3<=go_flag2;end
                          else begin
                          		go_flag1<=0;
                                  go_flag2<=0;
                                  go_flag3<=0;
                          end
          end
          default:begin
              cnt1<=0;
              distance_mm1<=0;
                              distance_mm1_1<=0;
                  distance_mm1_2<=0;
                  distance_mm1_3<=0;
              step<=1;
              go_flag1<=0;
              go_flag2<=0;
              go_flag3<=0;
          end
      endcase
      end//1
   end
      else if(flag_back) begin 
          if(group_pulse)begin
      step<=2;
      cnt1<=0;
      cnt2<=0;
      trig_1<=0;
      trig_2<=0;
      end
   else    if(step!=2)
      		step<=2;
    else    if(step==2) begin//4
      	case(cstate2)
          IDLE:if(cnt2<WAIT_TIME-1) cnt2<=cnt2+1;
          else cnt2<=0;
          SEND_TRIG:if(cnt2<TRIG_TIME-1) begin cnt2<=cnt2+1;
              trig_2<=1;
          end
          else begin cnt2<=0;
              trig_2<=0;
          end
          WAIT_ECHO:if(echo2_2==1) cnt2<=0;
          else begin if(cnt2<OVER_TIME-1)cnt2<=cnt2+1;
              else begin cnt2<=0;
                  distance_mm2<=0;
                  step<=2;
              end
          end
          MEASURE:cnt2<=cnt2+1;
          CALC: begin distance_mm2<=((cnt2+1)*223)>>16;
                          distance_mm2_1<=distance_mm2;
                  distance_mm2_2<=distance_mm2_1;
                  distance_mm2_3<=distance_mm2_2;
              cnt2<=0;
              step<=2;		if(distance_vld_back) begin
                      back_flag1<=(distance_2>OBSTACLE_OFF && mid_value2[0]>OBSTACLE_OFF ) ;
      					back_flag2<=back_flag1;
      					back_flag3<=back_flag2;end 
                          else begin
                          back_flag1<=0;
                          back_flag2<=0;
                          back_flag3<=0;
                          end
          end
          default:begin
              cnt2<=0;
              distance_mm2<=0;
                                              distance_mm2_1<=0;
                  distance_mm2_2<=0;
                  distance_mm2_3<=0;
              step<=2;
              back_flag1<=0;
              back_flag2<=0;
              back_flag3<=0;
          end
      endcase
      end//4
    end
  end 
  end
  wire [15:0]distance_1_max=(distance_mm1_1>distance_mm1_2)?
  						(distance_mm1_1>distance_mm1_3?distance_mm1_1:distance_mm1_3)://1>2的情况下1还>3，那就是1最大
                          (distance_mm1_2<distance_mm1_3?distance_mm1_3:distance_mm1_2);//1<2的情况下2＜3，2＞3
  wire [15:0]distance_2_max=(distance_mm2_1>distance_mm2_2)?
  						(distance_mm2_1>distance_mm2_3?distance_mm2_1:distance_mm2_3)://1>2的情况下1还>3，那就是1最大
                          (distance_mm2_2<distance_mm2_3?distance_mm2_3:distance_mm2_2);//1<2的情况下2＜3，2＞3
  wire [15:0]distance_1_min=(distance_mm1_1<distance_mm1_2)?
  						(distance_mm1_2<distance_mm1_3?                        //1小于2的情况下2小于3，最小的就是1
                          distance_mm1_1:(distance_mm1_1>distance_mm1_3)		//1小于2的情况下2大于3，那就比较1和3
                          ?distance_mm1_3:distance_mm1_1):
                          (distance_mm1_2>distance_mm1_3?distance_mm1_3:distance_mm1_2);//1大于2的情况下2大于3，最小的就是3，否则最小的就是2
  wire [15:0]distance_2_min=(distance_mm2_1<distance_mm2_2)?
  						(distance_mm2_2<distance_mm2_3?                        //1小于2的情况下2小于3，最小的就是1
                          distance_mm2_1:(distance_mm2_1>distance_mm2_3)		//1小于2的情况下2大于3，那就比较1和3
                          ?distance_mm2_3:distance_mm2_1):
                          (distance_mm2_2>distance_mm2_3?distance_mm2_3:distance_mm2_2);//1大于2的情况下2大于3，最小的就是3，否则最小的就是2
  wire [15:0]distance_1_mid=(distance_mm1_1>distance_mm1_2)?
  						(distance_mm1_2>distance_mm1_3?distance_mm1_2:
                          	distance_mm1_1>distance_mm1_3?distance_mm1_3:distance_mm1_1):
                             (distance_mm1_2<distance_mm1_3?distance_mm1_2:
                          distance_mm1_1<distance_mm1_3?distance_mm1_3:distance_mm1_1);
   wire [15:0]distance_2_mid=(distance_mm2_1>distance_mm2_2)?
  						(distance_mm2_2>distance_mm2_3?distance_mm2_2:
                          	distance_mm2_1>distance_mm2_3?distance_mm2_3:distance_mm2_1):
                           (distance_mm2_2<distance_mm2_3?distance_mm2_2:
                          distance_mm2_1<distance_mm2_3?distance_mm2_3:distance_mm2_1);	
  wire [18:0]sum_all1={3'd0,mid_value1[0]}
  			    +{3'd0,mid_value1[1]}
                  +{3'd0,mid_value1[2]}
                  +{3'd0,mid_value1[3]}
                  +{3'd0,mid_value1[4]}
                  +{3'd0,mid_value1[5]}
                  +{3'd0,mid_value1[6]}                
                  +{3'd0,mid_value1[7]};//用于均值的和
  reg [15:0]mid_value1[7:0];//
  wire [18:0]sum_all2={3'd0,mid_value2[0]}
  			    +{3'd0,mid_value2[1]}
                  +{3'd0,mid_value2[2]}
                  +{3'd0,mid_value2[3]}
                  +{3'd0,mid_value2[4]}
                  +{3'd0,mid_value2[5]}
                  +{3'd0,mid_value2[6]}                
                  +{3'd0,mid_value2[7]};//用于均值的和
  reg [15:0]mid_value2[7:0];//
  assign distance_1=sum_all1>>3;
  assign distance_2=sum_all2>>3;
  always @ ( posedge clk_50m or posedge rst )
  begin 
  	if(rst) begin
      mid_value1[7]<=0;
      mid_value1[6]<=0;
      mid_value1[5]<=0;
      mid_value1[4]<=0;                                   
      mid_value1[3]<=0;
      mid_value1[2]<=0;
      mid_value1[1]<=0;
      mid_value1[0]<=0;
      mid_value2[7]<=0;
      mid_value2[6]<=0;
      mid_value2[5]<=0;
      mid_value2[4]<=0;                                   
      mid_value2[3]<=0;
      mid_value2[2]<=0;
      mid_value2[1]<=0;
      mid_value2[0]<=0;
      end
      else begin
      		if(cstate1==CALC) begin
      mid_value1[7]<=mid_value1[6];
      mid_value1[6]<=mid_value1[5];
      mid_value1[5]<=mid_value1[4];
      mid_value1[4]<=mid_value1[3];
      mid_value1[3]<=mid_value1[2];
      mid_value1[2]<=mid_value1[1];
      mid_value1[1]<=mid_value1[0];
      mid_value1[0]<=distance_1_mid;
      end
      else
      		if(cstate2==CALC) begin
      mid_value2[7]<=mid_value2[6];
      mid_value2[6]<=mid_value2[5];
      mid_value2[5]<=mid_value2[4];
      mid_value2[4]<=mid_value2[3];
      mid_value2[3]<=mid_value2[2];
      mid_value2[2]<=mid_value2[1];
      mid_value2[1]<=mid_value2[0];
      mid_value2[0]<=distance_2_mid; 
      end
      end
  end
  //之前三次一致有点重复了，我就删除了
  reg go_flag1;
  reg go_flag2;
  reg go_flag3;
  always @ (posedge clk_50m or posedge rst  )
  begin 
  	if(rst) begin
      distance_vld_go<=0;
      end
      else begin
      		if(flag_idle||flag_go) begin
              if(distance_vld_go==1) begin
              		if(distance_1>OBSTACLE_OFF && mid_value1[0]>OBSTACLE_OFF ) begin//慢速解除
                      distance_vld_go<=~(go_flag1 && go_flag2 && go_flag3); end
                          end
               else if(mid_value1[0]<OBSTACLE_ON) begin//快速报警
              			distance_vld_go<=1;
                          end
              end
      end
  end
  
  reg back_flag1;
  reg back_flag2;
  reg back_flag3;
  always @ (posedge clk_50m or posedge rst  )
  begin 
  	if(rst) begin
      distance_vld_back<=0;
      end
      else begin
      		if(flag_idle||flag_back) begin
              if(distance_vld_back==1) begin
              		if(distance_2>OBSTACLE_OFF&& mid_value2[0]>OBSTACLE_OFF ) begin
                      distance_vld_back<=~(back_flag1 && back_flag2 && back_flag3);
                          end
                          end
               else if(mid_value2[0]<OBSTACLE_ON) begin
              			distance_vld_back<=1;
                          end
              end
      end
  end
  
  
endmodule    //ULTRASONIC_SENSOR_cg
