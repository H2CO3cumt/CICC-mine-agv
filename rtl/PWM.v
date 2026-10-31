module PWM(
input wire [3:0] car_state,
input wire clk_50m,
input wire rst,
output reg pwm_left,
output reg pwm_right,
input wire heavy_load_state,
input wire en_1k,
input wire en,
output wire IN1,
output wire IN2,
output wire IN3,
output wire IN4,
input wire [3:0] error_mag,
input wire turn_dir,
input wire sensor_lock,
input wire enc_left_a,
input wire enc_left_b,
input wire enc_right_a,
input wire enc_right_b,
input wire mpu_dir,
input wire [3:0] error_mpu,
input wire sensor_lost,
input wire force_stop,
input wire distance_vld,
output reg [15:0] target_speed_l,
output reg [15:0] target_speed_r,
output reg [15:0] actual_speed_left,
output reg [15:0] actual_speed_right
);

  //----Code starts here: integrated by Robei-----
  
    //组合逻辑always@(*)不允许用复位
    parameter MAG_SPEED=10;
    parameter ERROR_SPEED=60;
    parameter IDLE=4'b0000;
    parameter GO=4'b0001;
    parameter BACK=4'b0010;
    parameter LEFT=4'b0100;
    parameter RIGHT=4'b1000;
    parameter PWM_FREQ=1000;//PWM的频率
    parameter CLK_FREQ=50_000_000;//输入的系统时钟频率
    parameter PERIOD=CLK_FREQ/PWM_FREQ;//PWM一个周期所经过的系统时钟周期，50_000;
    localparam DUTY_IDLE= 8'd0;
    //这都是50MS内编码器双边沿测量的结果，不是PWM值，别搞混了
    //localparam DUTY_LOAD_GOBACK= 8'd82;//重载的前进/后退速度
    //localparam DUTY_GOBACK= 8'd120;// 空载的前进/后退速度
    //localparam DUTY_TURN_IN= 8'd40; //空载转弯内测速度
    //localparam DUTY_TURN_OUT= 8'd100;//空载转弯外侧速度
    //localparam DUTY_LOAD_TURN_IN  = 8'd30;//重载转弯内侧速度
    //localparam DUTY_LOAD_TURN_OUT = 8'd75;//重载转弯外侧速度
    localparam DUTY_LOAD_GOBACK= 10'd360;//重载的前进/后退速度
    localparam DUTY_GOBACK= 10'd400;// 空载的前进/后退速度
    localparam DUTY_TURN_IN= 10'd220; //空载转弯内测速度
    localparam DUTY_TURN_OUT= 10'd440;//空载转弯外侧速度
    localparam DUTY_LOAD_TURN_IN  = 10'd165;//重载转弯内侧速度
    localparam DUTY_LOAD_TURN_OUT = 10'd330;//重载转弯外侧速度
  parameter TRIM=0;
  reg lost_used;
  reg [27:0]lost_cnt;
  localparam lost_max=50_000_000;
  always @ (  posedge clk_50m or posedge rst)
  begin 
  	if(rst)begin
      lost_used<=0;
      lost_cnt<=0;
      end
      else begin
      		if(~sensor_lost||~en||~sensor_lock)begin
              lost_used<=0;
              lost_cnt<=0;
              end
              else begin	if(~lost_used)begin
              if(lost_cnt>=lost_max-1)begin
              			lost_used<=1;
                          lost_cnt<=0;
                          end 
                          else begin
                          lost_used<=0;
                          lost_cnt<=lost_cnt+1;
                          end
                          end
              	else begin
                  lost_used<=1;
                  lost_cnt<=lost_cnt;
                  end
                  end		
                  end
  end
  reg [3:0]error_used;
  always @ (*  )
  begin 
  	if(~lost_used && sensor_lost)
      		error_used=10;
              else begin
  	case(error_mag)
      0:error_used=0;
      1:error_used=1;
      2:error_used=2;
      3:error_used=6;
      4:error_used=10;
      5:error_used=7;
      6:error_used=6;
      7:error_used=7;
      default:error_used=7; 
      endcase
      end   
  end	
   // reg [15:0]target_speed_l;//目标的编码器上升/下降沿数量，不是最终的占空比
   // reg [15:0]target_speed_r;
    reg dir_left;//0是正转动，1是反转
    reg dir_right;
    reg toggle_flag;//现在没用了，本来我设计的是用它反复在01跳变来取0.5的效果，以实现不用corr_value和MPU6050来走直线，发现实现不了
    integer base_speed;//integer 默认是32位的有符号位，系统自动截取了前八位
    integer temp_speed_l;
    integer temp_speed_r;//用于临时存储左右轮转速，防止直接用target存储的时候溢出
    //目标编码生成区
    always@(*)begin//sensor_lock也就是循迹模式优先级最高，通过TRACKING_SENSOR传来的error_mag表示偏移量，turn_dir表示应该往哪转
    //类似于编码器测速，只不过是编码器换成了8路传感器
    	target_speed_l=0;
        target_speed_r=0;
        dir_left=0;
        dir_right=0;
        base_speed=0;
        temp_speed_l=0;
        temp_speed_r=0;
       if(sensor_lock)begin
        dir_left=0;
        dir_right=0;
         		if(lost_used||force_stop||distance_vld) begin
                     // 两侧目标速度同时清零
          base_speed =0;
          temp_speed_l =0;
          temp_speed_r=0;
          target_speed_l=0;
          target_speed_r=0;
      end          
  		/* else begin   case(error_used)
          4'd0: base_speed=(load_state==1)?180:190;
          4'd1: base_speed=(load_state==1)?170:180;
          4'd2: base_speed=(load_state==1)?160:170;
          4'd3: base_speed=(load_state==1)?150:160;
          4'd4: base_speed=(load_state==1)?140:150;
          4'd5: base_speed=(load_state==1)?130:140;
          4'd6: base_speed=(load_state==1)?120:130;
          4'd7: base_speed=(load_state==1)?115:120;
          4'd8: base_speed=(load_state==1)?110:115;
          4'd9: base_speed=(load_state==1)?100:105;
          4'd10: base_speed=(load_state==1)?80:90;         
          default: base_speed=0;
      endcase*/
      else begin   case(error_used)
          4'd0: base_speed=(heavy_load_state==1)?280:290;
          4'd1: base_speed=(heavy_load_state==1)?270:280;
          4'd2: base_speed=(heavy_load_state==1)?260:270;
          4'd3: base_speed=(heavy_load_state==1)?250:260;
          4'd4: base_speed=(heavy_load_state==1)?240:250;
          4'd5: base_speed=(heavy_load_state==1)?230:240;
          4'd6: base_speed=(heavy_load_state==1)?220:230;
          4'd7: base_speed=(heavy_load_state==1)?215:220;
          4'd8: base_speed=(heavy_load_state==1)?210:215;
          4'd9: base_speed=(heavy_load_state==1)?200:205;
          4'd10: base_speed=(heavy_load_state==1)?180:190;         
          default: base_speed=0;
      endcase
      end
        		if(error_used==0)begin
                temp_speed_l=base_speed-TRIM;
                temp_speed_r=base_speed;
                            target_speed_l=(temp_speed_l<0)?0:((temp_speed_l>4000)?4000:temp_speed_l);
                           target_speed_r=(temp_speed_r<0)?0:((temp_speed_r>4000)?4000:temp_speed_r);
                end
                else begin if(turn_dir==1)//左转，右轮转速比左轮快
                		begin
                        temp_speed_l=base_speed-error_used*MAG_SPEED-TRIM;
                        temp_speed_r=base_speed+error_used*MAG_SPEED;
                        end
                        else begin//右转
                        temp_speed_l=base_speed+error_used*MAG_SPEED-TRIM;
                        temp_speed_r=base_speed-error_used*MAG_SPEED;                    
                        end
                                    target_speed_l=(temp_speed_l<0)?0:((temp_speed_l>4000)?4000:temp_speed_l);
                           target_speed_r=(temp_speed_r<0)?0:((temp_speed_r>4000)?4000:temp_speed_r);
                       end
  
        end
        else begin
        case(car_state)
        IDLE:begin
    	target_speed_l=0;
        target_speed_r=0;
        dir_left=0;
        dir_right=0;     
        end
      
   
      GO:begin//这里我不管偏左还是偏右，我都是左轮减去corr_value，右轮加上corr_value，因为corr_value本身就是带符号的数据
        //它的运算逻辑与error_mpu以及编码器测速是分开的，就表示编码器在一段时间内测量的总脉冲数来代替路程
        //如果corr_value是正数的话表示左轮走的路程多，那我就得左减右加，负数的话是右轮走的路程多，那我得左加右减，减一个负数
        //就相当于加一个正数，逻辑没问题
        //但是error_mpu就不一样了，偏右我就得左轮减速，偏左我就得右轮减速，逻辑不同的关键就在于我有mpu_dir来限制了error_mpu的方向
        //而corr_value不同，正负都可能存在
            dir_left=0;
        dir_right=0; 
        base_speed=(heavy_load_state==1)?DUTY_LOAD_GOBACK:DUTY_GOBACK;
        if(error_mpu==0)begin
        target_speed_r=base_speed+corr_value;
        target_speed_l=base_speed-TRIM-corr_value;
        end
        else begin
        			if(mpu_dir==1)//偏右了
                    begin
                    	/*target_speed_l=base_speed-TRIM-corr_value-error_mpu*ERROR_SPEED;//我觉得本质上就是PI算法的变形，corr_value是积分项，考虑的是总的偏差，error_mpu是比例项，考虑的是瞬时的偏差
                    	target_speed_r=base_speed+corr_value;*/
                     target_speed_l=base_speed-error_mpu*ERROR_SPEED;//-error_mpu*ERROR_SPEED-TRIM-corr_value;
                                          target_speed_r=base_speed;//+error_mpu*ERROR_SPEED;//+corr_value;
                        end
                        else if(mpu_dir==0)begin
                     /* target_speed_r=base_speed+corr_value-error_mpu*ERROR_SPEED;
                    	target_speed_l=base_speed-TRIM-corr_value;*/
                                        target_speed_l=base_speed;//+error_mpu*ERROR_SPEED-corr_value;
                                          target_speed_r=base_speed-error_mpu*ERROR_SPEED;//-error_mpu*ERROR_SPEED+corr_value;
                      
                        end
                        
        end
        end
        LEFT:begin
    	target_speed_r=(heavy_load_state==1)?DUTY_LOAD_TURN_OUT:DUTY_TURN_OUT;
        target_speed_l=(heavy_load_state==1)?DUTY_LOAD_TURN_IN:DUTY_TURN_IN;
        dir_left=0;
        dir_right=0;    
        end
        RIGHT:begin
    	target_speed_l=(heavy_load_state==1)?DUTY_LOAD_TURN_OUT:DUTY_TURN_OUT;
        target_speed_r=(heavy_load_state==1)?DUTY_LOAD_TURN_IN:DUTY_TURN_IN;
        dir_left=0;
        dir_right=0;
        end
        BACK:begin
            dir_left=1;
        dir_right=1; 
        base_speed=(heavy_load_state==1)?DUTY_LOAD_GOBACK:DUTY_GOBACK;
        if(error_mpu==0)begin
        target_speed_r=base_speed+corr_value-15;
        target_speed_l=base_speed-corr_value;
        end
        else begin
        			if(mpu_dir==0)//
                    begin
                    	target_speed_l=base_speed-corr_value-error_mpu*ERROR_SPEED;
                    	target_speed_r=base_speed+corr_value+error_mpu*ERROR_SPEED*7;//-error_mpu*ERROR_SPEED;
                        end
                        else if(mpu_dir==1)begin//右轮太快了
                      target_speed_r=base_speed+corr_value-error_mpu*ERROR_SPEED;
                    	target_speed_l=base_speed-corr_value+error_mpu*ERROR_SPEED*7;//-error_mpu*ERROR_SPEED;
                        end
                        
        end
        end
        default:begin
    	target_speed_l=0;
        target_speed_r=0;
        dir_left=0;
        dir_right=0;     
        end
        endcase
        end
    end
    //编码器读取区
    //我本来设计的是只读取上升沿跳变，但是后来为了达到更高的精度，就改成了下降沿跳变也读取
    reg enc_left_r1,enc_left_r2;
    reg enc_right_r1,enc_right_r2;
    always @ (posedge clk_50m  )
    begin 
    	enc_left_r1<=enc_left_a;
        enc_left_r2<=enc_left_r1;
        	enc_right_r1<=enc_right_a;
        enc_right_r2<=enc_right_r1;//打拍，统一时钟域
    end
    wire pulse_left=enc_left_r2^enc_left_r1;
    wire pulse_right=enc_right_r2^enc_right_r1;
    reg [15:0]pulse_cnt_right;//当前周期内编码器的脉冲数
    reg [15:0]pulse_cnt_left;
   // reg [15:0]actual_speed_left;
   // reg [15:0]actual_speed_right;
    localparam SAMPLE_TIME=2_500_000;//编码器的周期是50ms
    reg [23:0]sample_timer;//判断有没有到达周期
    always @ (posedge clk_50m or posedge rst  )
    begin 
    	if(rst)begin
        pulse_cnt_right<=0;
        pulse_cnt_left<=0;
        actual_speed_left<=0;
        actual_speed_right<=0;
        sample_timer<=0;
        toggle_flag<=0;
        end
        else begin
        if(sample_timer<SAMPLE_TIME-1)begin
        if(pulse_left) pulse_cnt_left<=pulse_cnt_left+1;//意思就是一个sample周期内只要检测到pulse信号就对相应的pulse_cnt+1处理
        if(pulse_right) pulse_cnt_right<=pulse_cnt_right+1;
        sample_timer<=sample_timer+1;
        toggle_flag<=toggle_flag;
        end
        else if(sample_timer==SAMPLE_TIME-1)begin//sample周期的最后一个时钟进行左移两位也就是乘4处理，是为了后续达到更高的计算精度
        actual_speed_left<=pulse_cnt_left<<2;
        actual_speed_right<=pulse_cnt_right<<2;
        pulse_cnt_left<=0;
        pulse_cnt_right<=0;
        sample_timer<=0;//重新开始新的sample周期
        toggle_flag<=~toggle_flag;
        end
        end
    end
    //直线累计误差,我感觉没啥用，还不如直接让CORR_MAAX=0效果好
    localparam GOBACK_ERR_DEBAND=32;//误差死区
    localparam GOBACK_SHIFT=6;//修正系数，修正量等于误差/64，也就是2的6次方
    parameter GOBACK_CORR_MAX=100;
    localparam GOBACK_ERR_MAX=2000;//累计误差的限幅，最大就是2000;
    integer goback_error;//累计误差值
    integer corr_value;
    reg [3:0] lstate;//上一时钟的状态
  integer goback_sample_error;
  always @(*)
  begin
      // actual_speed使用pulse_cnt左移2位，
      // 所以这里也乘4，统一到target_speed的数值单位
      goback_sample_error = pulse_cnt_left;
      goback_sample_error = goback_sample_error - pulse_cnt_right;
      goback_sample_error = goback_sample_error * 4;
      // 左轮目标比右轮少TRIM，
      // 因此把TRIM加入误差基准
      goback_sample_error = goback_sample_error + TRIM+corr_value*2;
  end
    wire sample_tick=(sample_timer==2_499_999);
    always @ ( posedge clk_50m or posedge rst )
    begin//1 	
    if(rst)	begin//2
    			lstate<=IDLE;
                goback_error<=0;
                end //2
                else begin//3
                		lstate<=car_state;
                        	if(~en)begin//4
                            goback_error<=0;
                            end//4
                            else	if(sample_tick&&(error_mpu==0)) begin//en==1//5
                            		if(lstate==IDLE&& (car_state==GO||car_state==BACK)) begin//6
                                    goback_error<=0;
                                    end//6
                                    else 	if((lstate==GO&&car_state==GO)|| (car_state==BACK&&lstate==BACK))begin//7
    								goback_error<=goback_error+goback_sample_error;
                                    		if(goback_error+goback_sample_error>GOBACK_ERR_MAX)
                                            	goback_error<=GOBACK_ERR_MAX;
                                            else if(goback_error+goback_sample_error<-GOBACK_ERR_MAX)
                                            			goback_error<=-GOBACK_ERR_MAX;
                                    end//7
                                    else begin
                                    		goback_error<=0;//左转右转的时候给直线偏移量清0
                                    end
                                    
                            end//5
                end//3
    
    end//1
    always @ (*  )
    begin 
        corr_value=0;//GOBAND_ERR_DEBAND是死区，我只有累计误差值超过了死区才进行逻辑运算
        if(goback_error>GOBACK_ERR_DEBAND)begin//左轮快,后续我就得左轮减速右轮加速
        		corr_value=(goback_error-GOBACK_ERR_DEBAND)>>GOBACK_SHIFT;
                		if(corr_value>GOBACK_CORR_MAX)//超过了限幅 我就稳定在MAX
                        	corr_value=GOBACK_CORR_MAX;
                            else corr_value=corr_value;
                end
                else begin
      						if(goback_error<-GOBACK_ERR_DEBAND)		begin
                            	corr_value=-((-goback_error-GOBACK_ERR_DEBAND)>>GOBACK_SHIFT);//同上，现在是右轮快
                                if(corr_value<-GOBACK_CORR_MAX)
                                		corr_value=-GOBACK_CORR_MAX;
                                        else corr_value=corr_value;
                            end
                            else begin
                            corr_value=0;
                            end
                end
                
    end
    //PI控制器区
    localparam Kp=12;
    localparam Ki=2;
    reg [3:0]last_cstate;
    reg [15:0]abs_left;//左轮绝对误差
    reg err_left;//左轮的误差方向,0代表着目标速度大于实际速度,需要加速，1的话就是减速
    reg [15:0]integral_left;//左轮误差的累计值
    reg [15:0]p_term_l;//左轮比例计算结果
    reg [15:0]calc_duty_left;//左轮PI计算出的临时占空比
    reg [15:0]abs_right;
    reg err_right;
    reg [15:0]integral_right;
    reg [15:0]p_term_r;
    reg [15:0]calc_duty_right;
    reg [7:0]final_duty_l;
    reg [7:0]final_duty_r;//这才是最终的占空比
    always@(*)begin//误差计算逻辑
    	if(actual_speed_left>target_speed_l)begin
        abs_left=actual_speed_left-target_speed_l;
        err_left=1;//减速
        end
        else begin
        abs_left=target_speed_l-actual_speed_left;
        err_left=0;//加速   
        end
        	if(actual_speed_right>target_speed_r)begin
        abs_right=actual_speed_right-target_speed_r;
        err_right=1;//减速
        end
        else begin
        abs_right=target_speed_r-actual_speed_right;
        err_right=0;//加速   
        end
    end
    always@(posedge clk_50m or posedge rst)begin//1
    	if(rst)begin//2
        final_duty_l<=0;
        final_duty_r<=0;
        integral_right<=0;
        integral_left<=0;
    	last_cstate<=IDLE;
      calc_duty_right<=0;
      calc_duty_left<=0;
        end//2
        else begin //3
        if(sample_timer==SAMPLE_TIME-1) begin //4
        if (car_state != last_cstate) begin//5
                      integral_left  <= integral_left >> 2;//本来我设计的是状态转换的时候积分项直接清零，但是不丝滑
                      integral_right <= integral_right >> 2;
                      last_cstate <= car_state;
                  end     //5         
        	else begin	if(target_speed_l==0)//左轮区//6
                begin//7
                final_duty_l<=0;
               	integral_left<=0;
                calc_duty_left <= 0;
                end//7
            	else begin	if(err_left==0)//需要加速了//8
                    begin
                    if(integral_left+abs_left<4000)//积分限幅
                    //比如跑步的时候你前面几圈特别慢，你总不能最后几圈跑的飞快，你肯定得爆缸啊
                    	   integral_left<=integral_left+abs_left;
                        else begin
                        	integral_left<=4000;
                        end
                        calc_duty_left<=((integral_left*Ki)+abs_left*Kp)>>5;//我本来是>>3,但是因为我actual_speed<<2,相应的abs也<<2，为了保持数值的大小，我就把>>3变成了>>5
             //这里<<2的目的是在target_speed逻辑里实现更精确的计算，以实现不<<2的时候小数的情况，也就是提高控制参数的分辨性，而非编码器的分辨性
                        end
                   else //减速的时候
                begin
                		if(integral_left>abs_left)
                        	integral_left<=integral_left-abs_left;
                      else integral_left<=0;
                    if(integral_left*Ki>abs_left*Kp)
                    		calc_duty_left<=(integral_left*Ki-abs_left*Kp)>>5;
                     else calc_duty_left<=0;
    					end
                    end
                                                if(calc_duty_left>255)
                		final_duty_l<=255;
                 else final_duty_l<=calc_duty_left[7:0];
            			
    
                //
        		if(target_speed_r==0)//右轮区
                begin
                final_duty_r<=0;
               	integral_right<=0;
                calc_duty_right<=0;
                end
            else 	if(err_right==0)//需要加速了
                    begin
                    if(integral_right+abs_right<4000)
                    	   integral_right<=integral_right+abs_right;
                        else begin
                        	integral_right<=4000;
                        end
                        calc_duty_right<=((integral_right*Ki)+abs_right*Kp)>>5;
                        end
                   else //减速的时候 
                begin
                		if(integral_right>abs_right)
                        	integral_right<=integral_right-abs_right;
                      else integral_right<=0;
                    if(integral_right*Ki>abs_right*Kp)
                    		calc_duty_right<=(integral_right*Ki-abs_right*Kp)>>5;
                     else calc_duty_right<=0;
    
                    end
            			
                                                    if(calc_duty_right>255)
                		final_duty_r<=255;
                 else final_duty_r<=calc_duty_right[7:0];
    end
            end//3
                end//1
                
    end
    
    //PWM底层逻辑
    reg [15:0]pwm_cnt;
    wire [15:0]compare_left=(final_duty_l*PERIOD)/256;
    wire [15:0]compare_right=(final_duty_r*PERIOD)/256;
    always @ (posedge clk_50m or posedge rst  )
    begin 
    	if(rst)
        pwm_cnt<=0;
        else begin
        		if(pwm_cnt==PERIOD-1)
                pwm_cnt<=0;
                else pwm_cnt<=pwm_cnt+1;
        end
    end
    always@(posedge clk_50m or posedge rst)
    begin
    	if(rst)begin
        pwm_left<=0;
        pwm_right<=0;
        end
        else begin
      		if(lost_used)begin
              pwm_left<=0;
              pwm_right<=0;
              end
              else begin
        pwm_left<=(pwm_cnt<compare_left)?1:0;
        pwm_right<=(pwm_cnt<compare_right)?1:0;
      end
        end
    end
    assign IN1=(en==0||final_duty_l==0)?0:dir_left;
    assign IN2=(en==0||final_duty_l==0)?0:~dir_left;
    
    assign IN3=(en==0||final_duty_r==0)?0:~dir_right;
    assign IN4=(en==0||final_duty_r==0)?0:dir_right;
    //USB-UART测试
    
  
endmodule    //PWM
