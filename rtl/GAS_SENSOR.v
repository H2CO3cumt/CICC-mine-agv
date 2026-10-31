module GAS_SENSOR(
input wire clk_50m,
input wire rst,
input wire do,
output reg gas_alarm,
output reg force_stop
);

  //----Code starts here: integrated by Robei-----
  //因为cnt_warmup是从0开始计算的，我的设想是cnt_warmup==WARMUP_TIME-1时就达到了最大值，下一步应该在时序逻辑里cnt_warmup<=WARMUP_TIME-1;warmup_done<=1;,也就是说我第一次cnt_warmup==WARMUP_TIME-1的时候，啥都没发生，因为cnt_warmup<=WARMUP_TIME-1;warmup_done<=1;是非阻塞赋值，我下一时钟到达的时候我cnt_warmup第二次等于WARMUP_TIME-1，也就是相当于上一个周期结束了，这是第二个周期的第1秒（从1开始算），此时warmup_done为1，比较合理
  parameter WARMUP_TIME=250_000_000;//预热时间为5s，先让MQ4内部化学反应充分
  parameter FILTER_TIME=500_000;//数字滤波时间为10ms,尽量减少毛刺的影响，防止出现小车抽搐等现象的发生
  reg [27:0]cnt_warmup;//计算有没有过预热时间
  reg warmup_done;//1表示过了预热时间
  reg [19:0]cnt_filter;//计算有没有过数字滤波时间
  reg filter_done;//1表示过了数字滤波时间
  reg do1;
  reg do2;
  always @ ( posedge clk_50m or posedge rst )
  begin 
  	if(rst) begin
      do1<=1;
      do2<=1;
      end 
      else begin
      	do2<=do1;
      do1<=do;
      end
  end
  always @ ( posedge clk_50m or posedge rst )
  begin 
  	if(rst) begin
      cnt_warmup<=0;
      warmup_done<=0;
      end
      else if(~warmup_done) begin
      		if(cnt_warmup<WARMUP_TIME-1) 
      			cnt_warmup<=cnt_warmup+1;
            else begin
          		cnt_warmup<=WARMUP_TIME-1;
               warmup_done<=1;
              end
          end
  end
  always @ (posedge clk_50m or posedge rst  )
  begin 
  	if(rst) begin
      cnt_filter<=0;
      filter_done<=0;
      end
      else if(warmup_done) begin
      			if(~do2) begin
      				if(cnt_filter<FILTER_TIME-1) begin
                      cnt_filter<=cnt_filter+1;
                      filter_done<=filter_done;
  //上面filter_done<=filter_done的意思是还没完成数字滤波的时候filter_done保持原值，比如在系统刚开始的时候
  //瓦斯浓度达标，但是还没完成数字滤波，filter_done为复位时的0
  //瓦斯浓度超标，并且迟迟未退散，那我filter_done保持第一次数字滤波后的值，也就是1，防止发生脉冲信号，导致小车一抽一抽的
                      end
                   else begin
                  		cnt_filter<=0;
                       filter_done<=1;
                  end
      		end
              else begin
              		filter_done<=0;
                   cnt_filter<=0;
              end  
              end          
  end
  always @ ( posedge clk_50m or posedge rst )
  begin 
  	if(rst) begin
      		gas_alarm<=0;
          force_stop<=0;
      end
      else if(filter_done)begin
      		gas_alarm<=1;
          force_stop<=1;    
      end
      		else begin
      		gas_alarm<=gas_alarm;
          force_stop<=force_stop;    
      end  
  end
endmodule    //GAS_SENSOR
