module WEIGHT_SENSOR(
input wire clk_50m,
input wire rst,
input wire dout,
output reg sck,
output reg [23:0] data,
output reg heavy_load_state,
output reg data_vld,
output reg [1:0] load_event_state,
output reg load_event_toggle,
output reg [23:0] load_change_amount,
output reg settle_timeout_fault,
input wire [3:0] car_state,
output reg overload_state
);

  //----Code starts here: integrated by Robei-----
  
      localparam IDLE=3'b000;//重力传感器处于空闲状态
      localparam WAIT=3'b001;//要从WAIT进入SHIFT得先等待50个时钟周期，也就是1us
      localparam SHIFT=3'b010;//从传感器接受24个数据
      localparam GAIN=3'b011;
      localparam DONE=3'b100;
      localparam DONE_TIME_MAX=24'd5_000_000;//防止因断线而锁死在DONE
      localparam HEAVY_ON=600;
      localparam HEAVY_OFF=550;
      localparam OVERLOAD_ON=950;
      localparam OVERLOAD_OFF=900;
      reg [23:0]done_time;
        localparam [23:0] LOAD_EVENT_MIN =20; // 最小有效载荷变化量，当前按20g设置
        localparam [2:0] EVENT_CONFIRM_NUM =2;// 连续两组有效重量超过阈值后确认事件
        localparam [2:0] EVENT_CONFIRM_LAST = EVENT_CONFIRM_NUM-1; // 根据连续确认次数自动计算末次计数值
        localparam [23:0] SETTLE_DELTA_MAX =10;// 相邻两组重量变化不超过10g时认为趋于稳定
        localparam [2:0] SETTLE_CONFIRM_NUM=3;  // 连续三组稳定数据后重新设置基准值
        localparam [2:0] SETTLE_CONFIRM_LAST = SETTLE_CONFIRM_NUM -1; //根据稳定确认次数自动计算末次计数值
        localparam EVENT_ARMED =0; //等待新的装料或卸料事件状态
        localparam EVENT_SETTLE =1;// 事件已触发并等待重量稳定状态
        reg event_ctrl_state; // 载荷事件识别状态寄存器
        reg [23:0] load_ref_weight;//上一轮确认完成后的基准重量
        reg [23:0] load_prev_weight; // 上一组有效重量，用于判断当前重量是否稳定和是否产生新的基准重量
        reg load_ref_valid;//基准重量是否已经建立
        reg event_candidate_dir;// 重量变化的方向，1表示增加，0表示减少
        reg [2:0] event_confirm_cnt;//事件连续确认计数器
        reg [2:0] settle_confirm_cnt; // 重量稳定连续确认计数器
        wire [23:0] load_ref_delta;//当前重量与基准重量之间的绝对差值
        wire [23:0] load_step_delta;// 当前重量与上一组重量之间的绝对差值
        wire load_current_dir;  // 当前重量变化方向，1表示增加，0表示减少
      reg [23:0]data_r;//临时存储数据
      reg [7:0]delay_cnt;//由WAIT进入SHIFIT,延迟1us
      reg [7:0]shift_cnt;//SHIFT持续的时间，接受24个数据
      reg [2:0]cstate;
      reg [2:0]nstate;
      reg [8:0] tick_r;
      reg tick_10us;
      reg [23:0] raw_data;
      reg raw_vld;
      reg [23:0]d1,d2,d3;//3次前后挨着的原始值，用来干中值滤波
      reg [23:0]avg_buf[0:7];
      reg [23:0]zero_offset;//原始的带皮质量，需要拿raw_data-zero_offset才能得到去皮质量data
      reg [31:0]zero_sum;//开机前16个有效数据的总和
      reg [7:0]zero_cnt;
      reg zero_done;
      reg dout_r1,dout_r2,dout_r3;
      wire [23:0]med_val;
      wire [27:0]sum_all;//8次中值滤波后的值相加
      wire [23:0]avg_val;
      assign load_ref_delta=(data>load_ref_weight)?(data-load_ref_weight):(load_ref_weight-data);//当前重量与基准重量的变化量
      assign load_step_delta=(data>load_prev_weight)?(data-load_prev_weight):(load_prev_weight-data);//当前重量与上一组重量的变化量
      assign load_current_dir=(data>=load_ref_weight)?1:0;//当前的重量大于等于基准重量的时候就判定为1，也就是增加方向
      assign sum_all={4'd0,avg_buf[0]}+{4'd0,avg_buf[1]}+{4'd0,avg_buf[2]}+{4'd0,avg_buf[3]}+{4'd0,avg_buf[4]}+{4'd0,avg_buf[5]}+{4'd0,avg_buf[6]}+{4'd0,avg_buf[7]};
      assign avg_val=sum_all[26:3];
      assign med_val=(d3>d2)?
      						((d2>d1)?d2:(d3>d1)?d1:d3):
      						((d3>d1)?d3:(d2>d1)?d1:d2);  
      always @ (  posedge clk_50m or posedge rst)
      begin 
      		if(rst)
              		{dout_r3,dout_r2,dout_r1}<=3'b111;
                      else                          		{dout_r3,dout_r2,dout_r1}<={dout_r2,dout_r1,dout};
                      end
      always @ ( posedge clk_50m or posedge rst )
      begin 
      	if(rst) begin
          tick_10us<=0;
          tick_r<=0;
          end
          else begin
          if(cstate==SHIFT||cstate==GAIN) begin
          	if(tick_r<499)begin
              tick_r<=tick_r+1;
              tick_10us<=0;
              end
              else if(tick_r==499) begin
              		tick_10us<=1;
                      tick_r<=0;
              end
              end
              else begin
                     		tick_10us<=0;
                      tick_r<=0;   
              end
          
              end
          end
          
      always @ (posedge clk_50m or posedge rst  )
      begin 
      	if(rst)
          cstate<=IDLE;
          else cstate<=nstate;
      end
      always @ ( posedge clk_50m or posedge rst )
      begin 
      	if(rst)
          sck<=0;
          else if(cstate==SHIFT|| cstate==GAIN)begin
          			if(tick_10us)
          			sck<=~sck;
                      end
                      else sck<=0;
      end
      always @ ( * )
      begin
      case( cstate )
      IDLE:if(~dout_r3) nstate=WAIT;
      	else nstate=IDLE;
      WAIT:if(delay_cnt==49) nstate=SHIFT;
      	else nstate=WAIT;
      SHIFT:if(shift_cnt==23 && tick_10us && sck) nstate=GAIN;
      	else nstate=SHIFT;
      GAIN:if(tick_10us && sck)   nstate=DONE;
      		else nstate=GAIN;
      DONE :if(dout_r3==1) begin nstate = IDLE;
      end
      else begin
      	if(done_time>=DONE_TIME_MAX-1) begin
          		nstate=IDLE;
                  end
           else  begin nstate=DONE;
          end 
                  
      end
      
          default: nstate=IDLE;
      endcase
      end
      always @ (posedge clk_50m or posedge rst  )
      begin 
      	if(rst) begin
          		delay_cnt<=0;
                  done_time<=0;
                  end
           else begin  if(cstate==WAIT) begin
          						if(delay_cnt<49) 
                      delay_cnt<=delay_cnt+1;
                      			else  delay_cnt<=0;
                                  end
                  else delay_cnt<=0;//非WAIT状态，归0
                 if(cstate==DONE && dout_r3==0) done_time<=done_time+1;
              else done_time<=0;
                  end                
      end
      always@(posedge clk_50m or posedge rst)begin
      	if(rst)begin
          shift_cnt<=0;
          data_r<=0;
          raw_vld<=0;
          end
          else begin
          raw_vld<=0;
          case( cstate)
          IDLE:begin
          shift_cnt<=0;
          data_r<=0;
          end
          WAIT:begin
          shift_cnt<=0;
          end
          SHIFT:begin
          if(tick_10us)
          		if(sck) begin
          			if(shift_cnt<23) begin
          		data_r[23-shift_cnt]<=dout_r3;
              shift_cnt<=shift_cnt+1;
          end
          else begin
          		if(shift_cnt==23) begin
                  raw_data<=  {data_r[23:1], dout_r3} ^ 24'h800000;
                            // raw_data<=  {data[23],data_r[22:1], dout_r3};
                  shift_cnt<=24;
                  raw_vld<=1;
                  end
          end
          end
          end 
              default:  begin 
                               shift_cnt<=0;
          					end
          endcase
          end
      end
      reg filter_step;
      always @ ( posedge clk_50m or posedge rst )
      begin //1
      	if(rst)begin
          d3<=0;d2<=0;d1<=0;zero_offset<=0;zero_cnt<=0;avg_buf[0]<=0;avg_buf[1]<=0;avg_buf[2]<=0;avg_buf[3]<=0;avg_buf[4]<=0;avg_buf[5]<=0;avg_buf[6]<=0;avg_buf[7]<=0;heavy_load_state<=0;
          overload_state<=0;
          data<=0;
          zero_done<=0;zero_sum<=0;
          filter_step<=0;
          data_vld<=0;
          end
          else begin
          data_vld<=0;
          if(raw_vld)begin//3
    
    
          		d3<=d2;d2<=d1;d1<=raw_data;//第一级流水线
              filter_step<=1;
              end 
              else if(filter_step)begin//第二级流水线
              filter_step <= 1'b0;
                      avg_buf[7] <= avg_buf[6]; avg_buf[6] <= avg_buf[5];
                      avg_buf[5] <= avg_buf[4]; avg_buf[4] <= avg_buf[3];
                      avg_buf[3] <= avg_buf[2]; avg_buf[2] <= avg_buf[1];
                      avg_buf[1] <= avg_buf[0]; avg_buf[0] <= med_val;
              if(~zero_done)begin
              		if(zero_cnt<26)begin
                      data_vld<=0;
                      zero_cnt<=zero_cnt+1;
                      if(zero_cnt>=10)//先让前面的10个值生成一个avg_val
                      		zero_sum<=zero_sum+avg_val;
                              end
                              else  begin zero_done<=1;//完成
                              zero_offset<=zero_sum[27:4];
                              end
              end//零点校验还没完成
              else //零点校验完成之后
              begin data_vld<=1;
              		if(avg_val>zero_offset)
                      	data<=(avg_val-zero_offset)/413;//avg_val-zero_offset;
                    else data<=(zero_offset-avg_val)/413;//zero_offset-avg_val;
                  if(heavy_load_state==0) begin//迟滞比较，也就是判断从非重载到重载，从重载到非重载的阈值不是同一个数
                  		if(data>HEAVY_ON)
                          	heavy_load_state<=1;
                  end
                  else begin
                  		if(data<HEAVY_OFF)
                          heavy_load_state<=0;
  
                          
                  end
                           if(overload_state==0) begin
                          		if(data>OVERLOAD_ON)
                          		overload_state<=1;
                                  end
                          else if(data<OVERLOAD_OFF)
                          			overload_state<=0;
                  end
              end
          end//2
          end//1
    reg [7:0] stop_cnt;//比如说货舱漏了，一直漏矿物，我就得上报并且紧急停车
  always @ (posedge clk_50m or posedge rst  )
  begin 
  	if(rst) begin
                event_ctrl_state<=EVENT_ARMED; 
                load_ref_weight<=0; 
                load_prev_weight<=0;
                load_ref_valid<=0; 
                event_candidate_dir<=0; 
                event_confirm_cnt<=0; 
                settle_confirm_cnt<=0;
                load_event_state<=0; 
                load_event_toggle<=0; 
                load_change_amount<=0; 
              stop_cnt<=0;
              settle_timeout_fault<=0;
      end
      else begin
      			if(data_vld)begin
                  if(load_ref_valid==0) begin//还没建立基准
                  		load_ref_weight<=data;//默认的基准值是刚开始的data也就是0
                       load_prev_weight<=data;//上一组值，也就是0
                      load_ref_valid<=1;
                      event_ctrl_state<=EVENT_ARMED;//进入等待事件状态
                      event_candidate_dir<=0;//清除候选变化方向
                      event_confirm_cnt<=0;//清零事件确认计数器
                      settle_confirm_cnt<=0;//清零稳定确认计数器
                      stop_cnt<=0;
                  end//建立初始基准结束
                  else begin
                  		if(car_state!=4'b0000) begin
           load_prev_weight   <= data;
          event_confirm_cnt  <= 0;
          settle_confirm_cnt <= 0;
          stop_cnt           <= 0;
                          end
                 else begin 		case(event_ctrl_state)
                          EVENT_ARMED:begin
                          stop_cnt<=0;
                          load_prev_weight<=data;
                          settle_confirm_cnt<=0;//等待事件的时候清零稳定确认计数器
                          if(load_ref_delta>=LOAD_EVENT_MIN)//相对基准值的变化量达到了阈值
                          begin
                          		if(event_confirm_cnt==0)begin
                                  event_candidate_dir<=load_current_dir;
                                  event_confirm_cnt<=1;
                                  end
                               else begin
                              			if(event_candidate_dir==load_current_dir) begin//如果上一回的变化方向和这一回的相等
                                          		if(event_confirm_cnt>=EVENT_CONFIRM_LAST) begin//次数够了
                                                  event_ctrl_state<=EVENT_SETTLE;
                                                  settle_confirm_cnt<=0;
                                                  event_confirm_cnt<=0;
                                                  load_change_amount<=load_ref_delta;//变化量就是当前重量和基准量的差值
                                                  load_event_toggle<=~load_event_toggle;
                                                  		if(load_current_dir==1) begin
                                                          load_event_state<=2'b01;//锁存装料状态
                                                          end
                                                          else begin load_event_state<=2'b10;end
                                                  end
                                                  else event_confirm_cnt<=event_confirm_cnt+1;
                                          end
                                          else begin
                                          event_candidate_dir<=load_current_dir;//开启新一轮
                                          event_confirm_cnt<=1;
                                          end
                              end
                          end
                               else event_confirm_cnt<=0;//没到阈值
                          end
                          EVENT_SETTLE: begin
                          load_change_amount<=load_ref_delta;
                          load_prev_weight<=data;//供下一组判断稳定性
                          event_confirm_cnt<=0;
                if(load_step_delta<=SETTLE_DELTA_MAX&&settle_confirm_cnt>=SETTLE_CONFIRM_LAST) begin//稳定
                                          load_ref_weight<=data;
                                          event_ctrl_state<=EVENT_ARMED;
                                          settle_confirm_cnt<=0;
                                          stop_cnt<=0;
                                          end
                                          else  begin if(stop_cnt>=99) begin//没稳定
                                          					settle_timeout_fault<=1;//超时了还没稳定我就强行停车，疑似漏货
                                                              stop_cnt<=stop_cnt;end
                                                          else stop_cnt<=stop_cnt+1; 
                                          							
  												if(load_step_delta<=SETTLE_DELTA_MAX)
                                                  			settle_confirm_cnt<=settle_confirm_cnt+1;
                                                      else settle_confirm_cnt<=0;
                                                      end
                          end
                          default:begin
                          event_ctrl_state<=EVENT_ARMED;
                          event_candidate_dir<=0;
                          settle_confirm_cnt<=0;
                          event_confirm_cnt<=0;
                          stop_cnt<=0;
                          end
                          endcase
                          end
                  end
                  end
      end
  end
endmodule    //WEIGHT_SENSOR
