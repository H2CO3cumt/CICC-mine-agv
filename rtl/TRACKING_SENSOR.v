module TRACKING_SENSOR(
input wire clk_50m,
input wire rst,
input wire [7:0] direction_in,
input wire en_10k,
output reg [3:0] error_mag,
output reg sensor_lost,
output reg turn_dir,
output reg [3:0] track_confidence,
output reg [7:0] sensor_fault,
output reg [1:0] control_mode
);

  //----Code starts here: integrated by Robei-----
      // 1：传感器检测到轨迹时输出高电平
      // 0：传感器检测到轨迹时输出低电平
      parameter SENSOR_ACTIVE_HIGH=1;
      // en_10k为10 kHz时：
      // 200次  = 20 ms
      // 1000次 = 100 ms
      // 2000次 = 200 ms
   parameter [11:0] LOST_PREDICT_COUNT = 200;
   parameter [11:0] LOST_SEARCH_COUNT= 1000;
   parameter [11:0] LOST_STOP_COUNT= 2000;
  localparam [1:0]MODE_NORMAL=2'b00;//正常循迹模式
  localparam [1:0]MODE_DEGRADED=2'b01;//降级循迹，PWM适当减速
  localparam [1:0]MODE_RECOVERY=2'b10;//断轨预测恢复，PWM应当低速
  localparam [1:0]MODE_STOP=2'b11;//脱轨，应当安全停车
  //第一层，先三级打拍同步防止亚稳态
  reg [7:0]sync_1; 
  reg [7:0]sync_2;
  reg [7:0]sync_3;
  always @ (posedge clk_50m or posedge rst  )
  begin 
  		if(rst) begin
          sync_1<=8'b0000_0000;
   sync_2<=8'b0000_0000;
   sync_3<=8'b0000_0000;
          end else begin
          sync_1<=direction_in;
          sync_2<=sync_1;
          sync_3<=sync_2;
          end
  end
  wire [7:0]sensor_active=SENSOR_ACTIVE_HIGH?sync_3:~sync_3;//方便换灰度传感器的时候改
  //第二层，时域下五点多数滤波，之前的三次一致有点太苛刻了
  //每一路传感器都比较最近五次的结果，大于等于3次是1就认为是1
  reg [7:0]sample_0;
  reg [7:0]sample_1;
  reg [7:0]sample_2;
  reg [7:0]sample_3;
  reg [7:0]sample_4;
  reg [2:0]sample_fill_count;
  wire samples_ready=(sample_fill_count==5);
  always @ (posedge clk_50m or posedge rst  )
  begin 
  	if(rst) begin
      		    sample_0<=8'b0000_0000;
              sample_1<=8'b0000_0000;
              sample_2<=8'b0000_0000;
              sample_3<=8'b0000_0000;
              sample_4<=8'b0000_0000;
              sample_fill_count<=0;
      end
      else if(en_10k) begin
      sample_0<=sensor_active;
      sample_1<=sample_0;
      sample_2<=sample_1;
      sample_3<=sample_2;
      sample_4<=sample_3;
      			if(sample_fill_count<5)
                  	sample_fill_count<=sample_fill_count+1;
      end
  end
  integer i_major;
  integer majority_sum;
  reg [7:0]direction_majority;
  always @ ( * )
  begin 
  		direction_majority=8'b0000_0000;
          majority_sum=0;
          for(i_major=0;i_major<8;i_major++)begin
          majority_sum=sample_0[i_major]+sample_1[i_major]+sample_2[i_major]+sample_3[i_major]+sample_4[i_major];
          if(majority_sum>=3)
          		direction_majority[i_major]=1;
           else direction_majority[i_major]=0;
          end
    end
   //第三层，统计八路传感器的有效数量
  function [3:0]count_ones8;
  		input [7:0]value;
          integer k_count;
          begin
          count_ones8=0;
          for(k_count=0;k_count<8;k_count++)begin
          count_ones8=count_ones8+value[k_count];
          end
          end
              endfunction
    wire  [3:0]raw_active_count=count_ones8(direction_majority);
  //第四层，健康度诊断，每一路传感器具有四位健康值
  //12-15，健康；8-11轻度退化，4-7严重退化，0-3故障
  //诊断依据有两种，第一种是时间依据，固定窗口内频繁反转，可能是矿场内强电磁干扰，粉尘，接触不良
  //第二种是空间依据，内部传感器长期与相邻传感器矛盾
  //健康度不会因为一次异常直接变成故障，而是逐渐下降，反之亦然，也是正常后也是逐渐恢复
  reg [3:0]sensor_health[0:7];
  reg [7:0]toggle_count[0:7];
  reg [7:0]contradiction_count[0:7];
  reg [7:0]direction_last;
  reg [7:0]diag_window_count;
  integer i_diag;
  always @ ( posedge clk_50m or posedge rst )
  begin 
  	if(rst) begin
      direction_last<=8'b0000_0000;
      diag_window_count<=0;
      		for(i_diag=0;i_diag<8;i_diag++) begin
              sensor_health[i_diag]<=15;
              toggle_count[i_diag]<=0;
              contradiction_count[i_diag]<=0;
              end
      end 
      else if(en_10k) begin	if(~samples_ready) begin//还没填充好就先不更新
      			direction_last<=direction_majority;
               diag_window_count<=0;
              		for(i_diag=0;i_diag<8;i_diag++)begin
                      		toggle_count[i_diag]<=0;
                            contradiction_count[i_diag]<=0;
                      end
      end
      else begin
      direction_last<=direction_majority;
      		if(diag_window_count==8'hFF) begin//大约25.6ms更新一次健康度
              diag_window_count<=0;
              		for(i_diag=0;i_diag<8;i_diag++) begin
                      if(toggle_count[i_diag]>=80 || contradiction_count[i_diag]>=64) begin
                      			if(sensor_health[i_diag]>2) 
                                  sensor_health[i_diag]<=sensor_health[i_diag]-2;
                               else sensor_health[i_diag]<=0;
                      end
                      else if(toggle_count[i_diag]>=12|| contradiction_count[i_diag]>=24) begin
                      			if(sensor_health[i_diag]>1)
                                  	sensor_health[i_diag]<=sensor_health[i_diag]-1;
                               else sensor_health[i_diag]<=0;
                      end	//本窗口内表现正常，健康度恢复
                      		else  begin if(sensor_health[i_diag]<15)
                              		sensor_health[i_diag]<=sensor_health[i_diag]+1;
                                   else sensor_health[i_diag]<=15;
                                  end
                                  toggle_count[i_diag]<=0;
                                  contradiction_count[i_diag]<=0;//对应的清零
                      end//一次循环结束
              end
              else begin
              			diag_window_count<=diag_window_count+1;
                        for(i_diag=0;i_diag<8;i_diag++) begin
                      			if(direction_majority[i_diag]!=direction_last[i_diag]  && toggle_count[i_diag]!=8'hFF)//当前滤波值与上一周期不同，记录一次反转
                                  		toggle_count[i_diag]<=toggle_count[i_diag]+1;
                                     //检测孤立电平，只检测内部的那六个，边缘的2个不检测
                                  if(i_diag>0 && i_diag<7) begin
                                  		//孤立高电平，中间是1，两边是0，并且整个阵列还有其它的有效点
                                          if(direction_majority[i_diag]==1 && direction_majority[i_diag-1]==0 && direction_majority[i_diag+1]==0 && raw_active_count>=2 && contradiction_count[i_diag]!=8'hFF)
                                         	contradiction_count[i_diag]<=contradiction_count[i_diag]+1;
                                              //孤立的低电平
                                              if(direction_majority[i_diag]==0 && direction_majority[i_diag-1]==1 && direction_majority[i_diag+1] ==1 && raw_active_count>=2 && contradiction_count[i_diag]!=8'hFF)
                                         	contradiction_count[i_diag]<=contradiction_count[i_diag]+1;
                                  end
                      end
              end
      end
      
      end
  end
  //第五层，故障通道同构，把坏掉的变成好的
  //左右邻道同时为1时将中间通道变为1，左右邻道不同时为1时，中间的通道为0
  //对于边缘故障通道，不外推，直接置为0，避免虚构出不存在的轨迹
  reg [7:0]direction_reconstructed;
  integer i_recon;
  always @ ( * )
  begin 
  		direction_reconstructed=direction_majority;
          sensor_fault=8'b0000_0000;
          for(i_recon=0;i_recon<8;i_recon++) begin
          		if(sensor_health[i_recon]<4) begin//故障了
                  sensor_fault[i_recon]=1;
                if(i_recon>0 && i_recon<7)
              			direction_reconstructed[i_recon]= direction_majority[i_recon-1] && direction_majority[i_recon+1];
                    else direction_reconstructed[i_recon]=0;
                          end
                 
          end
  end
  //第六层，健康度加权质心
  //左4 左3 左2 左1  右1 右2 右3 右4
  //-7  -5  -3  -1   +1  +3  +5  +7
  //对于我们的健康权重，有:
  //health>=12 权重为3，>=8，权重为2，>=4,权重为1，>=0，权重就是0
  //加权质心:position=Σ(传感器位置 × 健康权重 × 状态)/Σ(健康权重 × 状态)
  //故障通道经过相邻重构后，只赋最低权重1
    integer weighted_sum_comb;
    integer valid_weight_sum_comb;
    integer track_position_comb;
    integer active_count_comb;
      integer segment_count_comb;
    integer fault_count_comb;
      integer suspect_count_comb;
    integer position_weight_comb;
      integer health_weight_comb;
   integer i_center;
   reg position_valid_comb;
  always @ ( * )
  begin 
          weighted_sum_comb     = 0;
          valid_weight_sum_comb = 0;
          track_position_comb   = 0;
          active_count_comb     = 0;
          segment_count_comb    = 0;
          fault_count_comb      = 0;
          suspect_count_comb    = 0;
          position_weight_comb  = 0;
          health_weight_comb    = 0;
          position_valid_comb   = 1'b0;
          //统计轨迹连续区域数量
          //0001_1000是1个连续区域，0101_0111是3个
          if(direction_reconstructed[7])//边缘是1的话必须得先算1个，不然就少了
          segment_count_comb=segment_count_comb+1;
          for(i_center=0;i_center<7;i_center++)begin
          		if(~direction_reconstructed[7-i_center] && direction_reconstructed[6-i_center])
                  		segment_count_comb=segment_count_comb+1;
  	        end
             for(i_center=0;i_center<8;i_center++)begin//根据传感器型号选权值
          case( i_center)
          0: position_weight_comb = 7;
          1: position_weight_comb = 5;
          2: position_weight_comb = 3;
           3:position_weight_comb = 1;
          4: position_weight_comb = -1;
          5: position_weight_comb = -3;
          6: position_weight_comb = -5;
          7: position_weight_comb = -7;//从0到7依次是从右往左
              default:   position_weight_comb = 0  ;
          endcase
          //根据通道的健康值选动态权重
          if(sensor_health[i_center]>=12) health_weight_comb=3; 
          else begin if(sensor_health[i_center]>=8) health_weight_comb=2;
                else  if(sensor_health[i_center]>=4) health_weight_comb=1;
              			else health_weight_comb=0;
                          end
          //统计故障通道数和严重退化的通道数
          		if(sensor_health[i_center]<4) fault_count_comb=fault_count_comb+1; 
                  else begin if(sensor_health[i_center]<8) suspect_count_comb=suspect_count_comb+1;end
                          //正常通道数
          		if(direction_reconstructed[i_center])begin  active_count_comb=active_count_comb+1;
                  		if(health_weight_comb==0 && i_center>0 && i_center <7) begin//故障通道的健康可信度是0
                          weighted_sum_comb=weighted_sum_comb+position_weight_comb;
                          valid_weight_sum_comb=valid_weight_sum_comb+1;
                          end
                          else if(health_weight_comb!=0) begin//正常或者退化的通道使用健康权重
                          		weighted_sum_comb=weighted_sum_comb+position_weight_comb*health_weight_comb;
                                  valid_weight_sum_comb=valid_weight_sum_comb+health_weight_comb;
                          end
  
                  end
                                          if(valid_weight_sum_comb!=0) begin//Q1定点加权质心，本来实际位置范围是-7到+7
                                          //error_mag是4位，上了Q2或者更高的就截断了
                                          //track_position_comb的实际值==track_position_comb÷2
                         //本来让weight_sum_com直接整除valid_weight_sum_comb会丢失小数位
                          track_position_comb=(weighted_sum_comb*2)/valid_weight_sum_comb;
                          position_valid_comb=1;
                          end
          end		
  end
  //第七层，时空轨迹置信度计算
  //置信度范围0-15，初试分数是15，根据以下情况扣分
  //  轨迹存在多个分离区域
  //  同时有效的传感器过多
  // 存在故障通道
  // 存在严重退化通道
  //  本次位置相对上次位置发生异常跳变
  //置信度位于12-15，8-11，4-7，0-3分别对应高 中 低  不可信
  integer confidence_comb;
  integer jump_abs_comb;
  integer fault_penalty_comb;
  integer suspect_penalty_comb;
  integer last_position;
  integer last_delta;
  reg history_valid;//已经保存过至少一次中高置信度轨迹
  always @ ( * )
  begin 
  	confidence_comb=0;
      jump_abs_comb=0;
      fault_penalty_comb=0;
      suspect_penalty_comb=0;
      		if(samples_ready && position_valid_comb) begin
              	confidence_comb=15;//初始就是15分
            //开始扣分了
          if(segment_count_comb==2) confidence_comb=confidence_comb-2;//两个连续区域扣两分
          else if(segment_count_comb>=3) confidence_comb=confidence_comb-6;//超过两个就扣六分
          if(active_count_comb>=7) confidence_comb=confidence_comb-3;//有效传感器太多了，可能是路口，宽线
          else if(active_count_comb>=5) confidence_comb=confidence_comb-1;
          fault_penalty_comb=fault_count_comb*2;
          if(fault_penalty_comb>=6) fault_penalty_comb=6;//传感器故障一个扣2分，最多扣6分
          confidence_comb=confidence_comb-fault_penalty_comb;
          suspect_penalty_comb=suspect_count_comb;
          	if(suspect_penalty_comb>=3) suspect_penalty_comb=3;//传感器退化一个扣1分，最多3分
                      confidence_comb=confidence_comb-suspect_penalty_comb;
                     if(history_valid) begin
            jump_abs_comb=track_position_comb-last_position;
          	if(jump_abs_comb<0) jump_abs_comb=-jump_abs_comb;
              //根据位置突变量扣分，偏差的越大扣的越多
              if(jump_abs_comb>=12) confidence_comb=confidence_comb-5;
              else if(jump_abs_comb>=10) confidence_comb=confidence_comb-4;
              		else if(jump_abs_comb>=8) confidence_comb=confidence_comb-3;
                      else if(jump_abs_comb>=6) confidence_comb=confidence_comb-2;
                      		else if(jump_abs_comb>=4) confidence_comb=confidence_comb-1;
                              end
                              else jump_abs_comb=0;//第一次有效轨迹没有历史参考，就不扣分
                  if(confidence_comb>15) confidence_comb=15;
                  else if(confidence_comb<0)confidence_comb=0;
                  end
  end
  //第八层，二阶轨迹趋势预测
  //位置变化量delta=当前轨迹位置-上一轨迹位置
  //位置变化加速度accel=当前delta-上一delta
  //预测位置=当前位置+delta+accel/2
  //这个模型用于矿井环境内传感器短时被粉尘遮挡或者断线的时候继续估计当前怎么运动
  //delta限制在-8到+8
  //accel限制在-4到+4
  integer delta_comb;
  integer accel_comb;
  integer predicted_position_comb;
  integer selected_position_comb;
  always @ (*  )
  begin 
  delta_comb=0;
  accel_comb=0;
  predicted_position_comb=track_position_comb;
  selected_position_comb=track_position_comb;
   if(history_valid) begin
  	if(confidence_comb>=8) begin //置信度>=8，才使用当前测量，否则用预测输出
  		 delta_comb=track_position_comb-last_position;
  	if(delta_comb>8) delta_comb=8;
      else if(delta_comb<-8) delta_comb=-8;
  accel_comb=delta_comb-last_delta;
  	if(accel_comb>4) accel_comb=4;
      else if(accel_comb<-4) accel_comb=-4;
  predicted_position_comb=track_position_comb+delta_comb+accel_comb/2;
  	if(predicted_position_comb>14) predicted_position_comb=14;
      else if(predicted_position_comb<-14) predicted_position_comb=-14;
      selected_position_comb=track_position_comb;
      end
    else begin  predicted_position_comb=last_position+last_delta;
  if(predicted_position_comb>14) predicted_position_comb=14;
      else if(predicted_position_comb<-14) predicted_position_comb=-14;
      selected_position_comb=predicted_position_comb;
      end
      end else begin//还没建立可靠历史，不能用二阶预测
      delta_comb=0;
      accel_comb=0;
  predicted_position_comb=track_position_comb;
  selected_position_comb=track_position_comb;
      end
  end
  function [3:0]position_to_error;
  		input integer position_value;
          integer abs_value;
          begin
          if(position_value>0) abs_value=position_value;
          else abs_value=-position_value;
                  if(abs_value>15) abs_value=15;
           position_to_error=abs_value[3:0];
          end
          endfunction
    //第九层，分级脱轨恢复和安全状态机
  //脱轨
  //先使用最近一次二阶预测位置继续纠偏，还没回到轨道上的话就按最后有效方向逐渐增加搜索误差，之后保持低速搜索
  //最后还没找到的话就sensor_lost置位为1，control_mode进入MODE_STOP
  reg [23:0]lost_count;
  reg last_turn_dir;
  reg [3:0]last_error_mag;
  integer predicted_position_hold;
  integer search_error_comb;
  always @ ( * )
  begin 
  	if(lost_count>LOST_PREDICT_COUNT) begin
      			search_error_comb=last_error_mag+(lost_count-LOST_PREDICT_COUNT)/100;
                  end
  		else search_error_comb=last_error_mag;
        if(search_error_comb>14) search_error_comb=14;//限幅
      else if(search_error_comb<0) search_error_comb=0;
  end
  //轨迹可用条件:已经完成五点采样&& 加权质心计算有效&&轨迹置信度不低于4
  wire usable_track=samples_ready && position_valid_comb && (confidence_comb>=4);
   always @(posedge clk_50m or posedge rst) begin
          if (rst) begin
   error_mag <= 0;
   sensor_lost <= 1;
  turn_dir<=0;
     track_confidence <=0;
     control_mode <= MODE_STOP;
   lost_count<= 0;
   last_turn_dir <=0;
     last_error_mag  <=0;
      last_position<=0;
    last_delta <=0;
   history_valid<=0;
  predicted_position_hold <= 0;end
  else if(en_10k)begin
  	if(~samples_ready) begin//五组滤波还没好
      error_mag<=0;
      sensor_lost<=1;
     track_confidence<=0;
  	control_mode<=MODE_STOP;
      lost_count<=0;
      end
      else if(usable_track) begin//当前存在可用轨迹
      			lost_count<=0;
              sensor_lost<=0;
              track_confidence<=confidence_comb;
              //寄存轨迹状态
              if(confidence_comb>=8) begin//只有中高置信度才更新预测轨迹，置信度为4-7时允许参与临时控制，但不能污染可靠历史
             last_position<= track_position_comb;
          		last_delta<=delta_comb;
                predicted_position_hold<=predicted_position_comb;
              history_valid<=1;
              end
              if(selected_position_comb<0) begin
              		turn_dir<=1;//轨迹位于系统的左侧
                    error_mag<=position_to_error(selected_position_comb);
                  last_turn_dir<=1;//轨迹位于系统的左侧
                   last_error_mag<=position_to_error(selected_position_comb);
              end 
              else if(selected_position_comb>0)begin	
                          		turn_dir<=0;//轨迹位于系统的右侧
                    error_mag<=position_to_error(selected_position_comb);
                                  last_turn_dir<=0;
                   last_error_mag<=position_to_error(selected_position_comb);
              end
              else begin
              		turn_dir<=last_turn_dir;
                  error_mag<=0;//中心时不覆盖最后搜索方向，防止刚到中心就丢线导致小车不知道往哪搜
              end 
            if(confidence_comb>=12 && suspect_count_comb==0 && fault_count_comb==0) 
          		control_mode<=MODE_NORMAL;
              else if(confidence_comb>=8) control_mode<=MODE_DEGRADED;
              		 else control_mode<=MODE_RECOVERY;
      end
      else begin//当前没有可用轨迹，置信度低于4
      		if(lost_count<LOST_STOP_COUNT)
              	lost_count<=lost_count+1;
             //第一级恢复
         		if(lost_count< LOST_PREDICT_COUNT) begin
              sensor_lost<=0;
              control_mode<=MODE_RECOVERY;
              track_confidence<=6;
              		if(predicted_position_hold<0) begin
                      turn_dir<=1;//左转
                      error_mag<=position_to_error(predicted_position_hold);
                      end
                      else      if(predicted_position_hold>0) begin
                      turn_dir<=0;//右转
                      error_mag<=position_to_error(predicted_position_hold);
                      end
                      else  begin turn_dir<=last_turn_dir;
                      error_mag<=last_error_mag;
                      end
              end
              //第二级恢复，沿最后一次搜索方向逐渐增大搜索误差，不能直接增大到最大值，防止甩尾
              else if(lost_count<LOST_SEARCH_COUNT) begin
              sensor_lost<=0;
              control_mode<=MODE_RECOVERY;
              track_confidence<=3;
              turn_dir<=last_turn_dir;
              error_mag<=search_error_comb;
              end
              else//第三级恢复保持低速强制定向搜索
               if(lost_count<LOST_STOP_COUNT) begin
              control_mode<=MODE_RECOVERY;
              		sensor_lost<=0;
              			turn_dir<=last_turn_dir;
                          error_mag<=14;
                          track_confidence<=1;
              end
              //第四级，这里我认为是轨迹暂时无法自主恢复，需要安全停车上报
              else begin
              sensor_lost<=1;
              control_mode<=MODE_STOP;
              track_confidence<=0;
              turn_dir<=last_turn_dir;
              error_mag<=0;
              end
      end
  end
     end
endmodule    //TRACKING_SENSOR
