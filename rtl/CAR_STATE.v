module CAR_STATE(
input wire en_1k,
input wire rst,
input wire [8:0] key_value,
output wire [3:0] car_state1,
output wire en,
input wire [8:0] key_cmd,
input wire clk_50m,
input wire [8:0] key_sensor,
input wire force_stop,
output reg bt_lock,
output reg sensor_lock,
input wire distance_vld_go,
input wire distance_vld_back,
input wire overload_state
);

  //----Code starts here: integrated by Robei-----
  
  //car_state;
  parameter IDLE=4'b0000;
  parameter GO=4'b0001;
  parameter BACK=4'b0010;
  parameter LEFT=4'b0100;
  parameter RIGHT=4'b1000;
  //key_value
  parameter DIGIT1=9'b111_111_110;
  parameter DIGIT2=9'b111_111_101;
  parameter DIGIT3=9'b111_111_011;
  parameter DIGIT4=9'b111_110_111;
  parameter DIGIT5=9'b111_101_111;
  parameter DIGIT6=9'b111_011_111;
  parameter DIGIT7=9'b110_111_111;
  parameter DIGIT8=9'b101_111_111;
  parameter DIGIT9=9'b011_111_111;
  parameter GO_BACK_MAX=25_000_000-1;
  parameter GO_IDLE_MAX=25_000_000-1;
  reg [27:0] cnt_go_back;//从任何挡位到BACK得是XX-IDLE-BACK的路径，防止炸管
  reg goback_flag;
  reg [27:0] cnt_go_idle;
  reg goidle_flag;//同上，只不过是从BACK到GO/LEFT/RIGHT需要触发这个逻辑,也就是说从IDLE到正转的任何状态都得经过0.5s
  reg [8:0]final_key;
  always @ ( posedge clk_50m or posedge rst )
  begin 
  	if(rst) begin
      		cnt_go_back<=0;
          goback_flag<=1;//车刚插上钥匙，肯定可以直接挂倒挡
              end
              else if(cstate==IDLE)begin
              				if(cnt_go_back<GO_BACK_MAX)//得先等0.5s
                              begin
                      		cnt_go_back<=cnt_go_back+1;
                              goback_flag<=0;
                              end
                          else   begin
              				cnt_go_back<=cnt_go_back;
              				goback_flag<=1;                        
                          end                         
                      end
              else begin
              if(cstate==BACK)  begin
              cnt_go_back<=0;
              goback_flag<=1;
              end
              else begin
              cnt_go_back<=0;
              goback_flag<=0;            
              end
              end
              			
  end
  always@(posedge clk_50m or posedge rst)begin
  if(rst) begin
  	cnt_go_idle<=0;
      goidle_flag<=1;
      end 
      else if(cstate==IDLE) begin
              				if(cnt_go_idle<GO_IDLE_MAX-1)//得先等0.5s
                              begin
                      		cnt_go_idle<=cnt_go_idle+1;
                              goidle_flag<=0;
                              end
                          else   begin
              				cnt_go_idle<=cnt_go_idle;
              				goidle_flag<=1;     
      end
      end
      else if(cstate==BACK)begin
  	cnt_go_idle<=0;
      goidle_flag<=0;    	
      end
      else begin
  	cnt_go_idle<=0;
      goidle_flag<=1;    
      end
  end
  always @ ( posedge clk_50m or posedge rst )
  begin 
  	if(rst)
      bt_lock<=0;
      else if(en_1k) begin
      			if(key_value==9'b111_111_011)
                  bt_lock<=1;
                  end
  end
  wire [8:0]selected_key=bt_lock?(key_cmd):(sensor_lock?9'b111_111_101:key_value);
  always @ ( posedge clk_50m or posedge rst )
  begin 
  	if(rst)
      sensor_lock<=0;
      else if(en_1k) begin
      			if(key_value==9'b011_111_111)
                  sensor_lock<=1;
                  end
  end
  always @ ( posedge clk_50m or posedge rst )
  begin 
  	if(rst)
      final_key<=DIGIT5;//默认停止；
      else begin
      		if(en_1k)
               begin 
              if(force_stop)
              final_key<=DIGIT5;
              else if(
              distance_vld_go&&(selected_key==DIGIT2||selected_key==DIGIT4||selected_key==DIGIT6)
  )
  final_key<=DIGIT5;                  
              						else if(distance_vld_back&&selected_key==DIGIT8)
                      					final_key<=DIGIT5;
                                         else if(overload_state)
                                      			final_key<=DIGIT5;
                      					else  begin 
                                          final_key<=selected_key;
                      						  end
                      end
              end
      end
  reg [3:0]cstate;//当前时钟状态；
  reg [3:0]nstate;//下一时钟状态；
  //wire [8:0]final_key=9'b0;//(key_cmd!=0)?key_cmd:key_value;//蓝牙命令优先于物理逻辑
  always @ ( posedge clk_50m or posedge rst)
  begin 
  	if(rst)
      cstate<=IDLE;
      else begin if(en_1k)
      cstate<=nstate;
      else cstate<=cstate;
      end
  end
  always @ (*)
  begin 
  case( final_key)
  	DIGIT2: if(goidle_flag)nstate=GO;//前进
      else nstate=IDLE;
      DIGIT4:if((cstate==IDLE&&goidle_flag)||cstate==GO||cstate==RIGHT||cstate==LEFT)nstate=LEFT;//左拐
      			else nstate=IDLE;
      DIGIT5:nstate=IDLE;//停止
      DIGIT6:if((cstate==IDLE&&goidle_flag)||cstate==GO||cstate==LEFT||cstate==RIGHT)nstate=RIGHT;//右拐
      			else nstate=IDLE;
      DIGIT8: if(goback_flag) nstate=BACK;
      else nstate=IDLE;//后退 
      default:nstate=cstate;
  endcase
  end
  assign car_state1=cstate;
  assign en=(cstate==GO||cstate==BACK||cstate==LEFT||cstate==RIGHT)?1:0;
  
  
endmodule    //CAR_STATE
