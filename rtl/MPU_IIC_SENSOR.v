module MPU_IIC_SENSOR(
input wire clk_50m,
input wire rst,
inout wire sda,
output wire scl,
output wire [15:0] raw_data,
output reg mpu_dir,
input wire en_100,
output reg [3:0] error_mpu,
output reg [3:0] ack_err_code,
input wire [3:0] car_state
);

  //----Code starts here: integrated by Robei-----
  // MPU6050 IIC通信
  localparam DEVICE_READ       = 8'hD1;
  localparam DEVICE_WRITE      = 8'hD0;
  localparam GYRO_ZH           = 8'h47;
  localparam PWR_MGMT_1        = 8'h6B;
  localparam PWR_MGMT_1_VAL    = 8'h00;
  localparam SMPLRT_DIV        = 8'h19;
  localparam SMPLRT_DIV_VAL    = 8'h07;
  localparam CONFIG1           = 8'h1A;
  localparam CONFIG1_VAL       = 8'h06;
  localparam GYRO_CONFIG       = 8'h1B;
  localparam GYRO_CONFIG_VAL   = 8'h18;
  localparam ACC_CONFIG        = 8'h1C;
  localparam ACC_CONFIG_VAL    = 8'h00;//地址
  localparam ST_RECOVER      = 4'd0;
  localparam ST_RECOVER_STOP = 4'd1;
  localparam ST_IDLE         = 4'd2;
  localparam ST_START        = 4'd3;
  localparam ST_SEND         = 4'd4;
  localparam ST_ACK          = 4'd5;
  localparam ST_RESTART      = 4'd6;
  localparam ST_READ         = 4'd7;
  localparam ST_MASTER_ACK   = 4'd8;
  localparam ST_STOP         = 4'd9;
  localparam ST_WAIT         = 4'd12;//状态
  //每个SLC分成四个
  reg [6:0] iic_tick_cnt;
  wire tick_iic;
  assign tick_iic = (iic_tick_cnt==124);//SCL是100KHZ,也就是一个SLC周期得计数50M/100K=500,再分成四份，也就是500/4=125
  always@(posedge clk_50m or posedge rst)begin
  	if(rst)
      		iic_tick_cnt<=0;
              else begin
              		if(tick_iic)
                      iic_tick_cnt<=0;
                      else iic_tick_cnt<=iic_tick_cnt+1;
              end   
  end
  reg sda_r1;
  reg sda_r2;
  always@(posedge clk_50m or posedge rst)begin//打拍,防止亚稳态
  		if(rst)begin
          		sda_r1<=1;
                  sda_r2<=1;
                  end
                  else begin
                  sda_r1<=sda;
                  sda_r2<=sda_r1;
                  end
          end
  reg sda_drive_low;
  reg scl_r;
  assign sda = sda_drive_low ? 1'b0 : 1'bz;
  assign scl = scl_r;
  reg [3:0] cstate;
  reg [1:0] phase;//0:低电平中心，1：上升沿，2：高电平中心，3：下降沿，把一个SLC周期平分为4个脉冲信号
  reg [3:0] recover_count;//统计恢复脉冲，0-8，共9个
  reg [11:0] wait_count;//统计10ms等待时间
  reg [2:0] bit_count;
  reg [7:0] byte_tx;//发送移位缓冲，就是发送第几个bit
  reg [7:0] byte_rx;//接收移位缓冲，这俩本质上就是形成一个8选1多路选择器
  reg [4:0] flow_step;//表示一次大循环里内部进行到了第几个环节
  reg [4:0] times;//表示进行到第几个大循环，0-4是初始化5个寄存器，5是读取MPU6050传来的高低位数据
  reg master_ack_low;//1的话就是主机拉低SDA,发送ACK,0的话就是主机释放了SDA，发送NACK
  reg transaction_ok;//记录当前循环是否成功
  reg [7:0] mpu_zl_read;
  reg [7:0] mpu_zh_read;
  reg [15:0] raw_data_r;
  reg raw_data_vld_r;
  reg [7:0] raw_data_vld_count;
  reg [15:0] pending_raw_data;//读取完高低字节后，不会立刻输出，而是先存入pending_raw_data，只有完成NACK,STOP,确认总线
  //恢复正常后，才把数据发到raw_data_r，不会出现刚开始那种高位是新的，低位是旧的情况
  reg pending_sample;
  reg i2c_init_done;
  reg [7:0] cfg_reg_addr;
  reg [7:0] cfg_reg_data;
  always @(*)//根据不同循环阶段选择不同的地址
  begin
      cfg_reg_addr = PWR_MGMT_1;//防止锁存器
      cfg_reg_data = PWR_MGMT_1_VAL;
      case(times)
          5'd0: begin
              cfg_reg_addr = PWR_MGMT_1;
              cfg_reg_data = PWR_MGMT_1_VAL;
          end
          5'd1: begin
              cfg_reg_addr = SMPLRT_DIV;
              cfg_reg_data = SMPLRT_DIV_VAL;
          end
          5'd2: begin
              cfg_reg_addr = CONFIG1;
              cfg_reg_data = CONFIG1_VAL;
          end
          5'd3: begin
              cfg_reg_addr = GYRO_CONFIG;
              cfg_reg_data = GYRO_CONFIG_VAL;
          end
  
          5'd4: begin
              cfg_reg_addr = ACC_CONFIG;
              cfg_reg_data = ACC_CONFIG_VAL;
          end
  
          default: begin
              cfg_reg_addr = GYRO_ZH;
              cfg_reg_data = 8'h00;
          end
      endcase
  end
  always @(posedge clk_50m or posedge rst)
  begin
      if(rst) begin
          cstate <= ST_RECOVER;//默认恢复
          phase<=0;
          recover_count<=0;
          wait_count<=0;
          bit_count<=7;
          byte_tx<=0;
          byte_rx<=0;
          flow_step<=0;
          times<=0;
          master_ack_low<=0;
          transaction_ok<=1;
          scl_r<=1;
          sda_drive_low<=0;
          mpu_zh_read<=0;
          mpu_zl_read<=0;
          raw_data_r<=0;
          raw_data_vld_r<=0;
          raw_data_vld_count<=0;
          pending_raw_data<=0;
          pending_sample<=0;
          i2c_init_done<=0;
          ack_err_code<=0;
      end
      else begin
          // raw_data_vld只保持一个50MHz时钟周期
          raw_data_vld_r <= 1'b0;
          if(tick_iic) begin
              case(cstate)
                  ST_RECOVER: begin
                      case(phase)
                          0: begin
                              scl_r<=0;
                              sda_drive_low<=0;//SCL拉低，SDA释放
                              phase<=1;
                          end
                          1: begin
                              scl_r<=1;//拉高，形成恢复上升沿
                              sda_drive_low<=0;
                              phase<=2;
                          end
                     2:begin
                              scl_r<=1;//保持高电平
                              sda_drive_low <= 1'b0;
                              phase <= 2'd3;
                          end
  
                          3: begin
                              scl_r<=0;//拉低，形成下降沿
                              sda_drive_low<=0;
                              if(recover_count==8) begin
                                  recover_count<=0;
                                  phase<=0;
                                  cstate<=ST_RECOVER_STOP;
                              end
                              else begin
                                  recover_count <= recover_count +1;
                                  phase<=0;//意思是得经历9个这样的SCL周期信号才能从RECOVER进入RECOVER_STOP;
                              end
                          end
                      endcase
                  end
                  ST_RECOVER_STOP:begin
                      case(phase)
                          0: begin
                              scl_r<=0;
                              sda_drive_low<=1;//SCL低的时候拉低SDA
                              phase<=1;
                          end
                          1: begin
                              scl_r<=1;
                              sda_drive_low<=1;//SCL高的时候读取
                              phase<=2;
                          end
                          2:begin
                              scl_r<=1;
                              sda_drive_low<=0;//SCL高的时候释放SDA,让他由低变高，正好是STOP的标准条件
                              phase<=3;
                          end
                          3:begin
                              scl_r<=1;
                              sda_drive_low<=0;
                              phase<=0;
                              cstate<=ST_IDLE;
                          end
                      endcase
                  end
                  ST_IDLE: begin
                      scl_r<=1;
                      sda_drive_low<=0;//SCL和SDA都是高
                      phase<=0;
                      bit_count<=7;
                      byte_rx<=0;
                      transaction_ok<=1;
                      // 如果空闲时SDA仍为低，再进行一次总线恢复
                      if(sda_r2==0) begin
                          recover_count<=0;
                          cstate<=ST_RECOVER;
                      end
                      else begin
                          byte_tx <= DEVICE_WRITE;
                          // 0-4为初始化，5为读取
                          if(times<5)
                              flow_step<=0;
                          else
                              flow_step<=10;
                          cstate <= ST_START;
                      end
                  end
                  ST_START: begin//SCL高电平的时候SDA得从高变低
                      case(phase)
                          0: begin
                              scl_r<=1;
                              sda_drive_low<=0;//两个都是高
                              phase<=1;
                          end
                          1:begin
                              scl_r<=1;
                              sda_drive_low<=1;//SCL高的时候SDA变低，这是START的开始信号
                              phase<=2;
                          end
                          2:begin
                              scl_r<=0;
                              sda_drive_low<=1;//开始信号产生后再把SCL拉低，准备改变SDA
                              phase<=3;
                          end
                          2'd3: begin
                              scl_r<=0;
                              phase<=0;
                              bit_count<=7;
                              cstate<=ST_SEND;
                          end
                      endcase
                  end
                  ST_SEND: begin
                      case(phase)
                          0: begin
                              scl_r<=0;
                              if(byte_tx[bit_count] ==0)
                                  sda_drive_low <=1;
                              else
                                  sda_drive_low<=0;
                              phase<=1;
                          end
                          1: begin
                              scl_r<=1;//从机在SCL高电平的时候读取SDA
                              phase<=2;
                          end
                          2: begin
                              scl_r<=1;
                              phase<=3;
                          end
  
                          3: begin
                              scl_r<=0;
  
                              if(bit_count==0) begin
                                  phase<=0;
                                  cstate<=ST_ACK;//为下一阶段做好准备
                              end
                              else begin
                                  bit_count<=bit_count-1;//还没发完
                                  phase<=0;
                              end
                          end
                      endcase
                  end
                  ST_ACK: begin//ACK是0，NACK是1
                      case(phase)
                          0:begin
                              scl_r<=0;
                              sda_drive_low<=0;//主机释放SDA控制权
                              phase<=1;
                          end
                          1: begin
                              scl_r<=1;
                              phase<=2;
                          end
                          2: begin
                              scl_r<=1;
                              phase<=3;
                          end
                          3: begin
                              scl_r<=0;
                              sda_drive_low<=0;
                              phase<=0;
                              // 没收到ACK
                              if(sda_r2 ==1) begin
                                  transaction_ok<=0;
                                  pending_sample<=0;
                                  // 保留第一次错误
                                  if(ack_err_code == 0) begin
                                      case(flow_step)
                                          0, 10:
                                              ack_err_code<=1;
                                          1,11:
                                              ack_err_code<=2;
                                          2:
                                              ack_err_code<=3;
                                          13:
                                              ack_err_code<=4;
                                          default:
                                              ack_err_code<=15;
                                      endcase
                                  end
                                  cstate <= ST_STOP;
                              end
                              else begin
                                  case(flow_step)
                                      // 初始化：发送设备写地址成功
                                      5'd0: begin
                                          byte_tx<=cfg_reg_addr;
                                          bit_count<=7;
                                          flow_step<=1;
                                          cstate<=ST_SEND;
                                      end
  
                                      // 初始化：发送寄存器地址成功
                                      1: begin
                                          byte_tx<=cfg_reg_data;
                                          bit_count<=7;
                                          flow_step<=2;
                                          cstate<=ST_SEND;
                                      end
                                      // 初始化：发送配置数据成功
                                      2: begin
                                          cstate<=ST_STOP;
                                      end
                                      // 读取：发送设备写地址成功
                                      10: begin
                                          byte_tx<=GYRO_ZH;
                                          bit_count<=7;
                                          flow_step<=11;
                                          cstate<=ST_SEND;
                                      end
  
                                      // 读取：发送0x47成功，产生重复START
                                      11: begin
                                          flow_step<=12;
                                          cstate<=ST_RESTART;
                                      end
                                      // 读取：发送设备读地址成功
                                      13: begin
                                          byte_rx<=0;
                                          bit_count<=7;
                                          flow_step<=14;
                                          cstate<=ST_READ;
                                      end
  
                                      default: begin
                                          transaction_ok<=0;
                                          cstate <=ST_STOP;
                                      end
                                  endcase
                              end
                          end
                      endcase
                  end
                  ST_RESTART: begin//和普通START的区别是不产生STOP
                      case(phase)
                          0: begin
                              scl_r<=0;
                              sda_drive_low<=0;//SCL低的时候释放SDA
                              phase<=1;
                          end
                          1: begin
                              scl_r<=1;
                              sda_drive_low<=0;//SCL拉高的时候SDA继续保持释放也就是高电平
                              phase<=2;
                          end
                          2: begin
                              scl_r<=1;
                              sda_drive_low<=1;//SDA由高变低，产生START
                              phase <= 2'd3;
                          end
                          3: begin
                              scl_r<=0;//SCL变低，准备下一阶段
                              sda_drive_low<=1;
                              byte_tx <= DEVICE_READ;//读MPU6050传来的
                              bit_count<=7;
                              flow_step<=13;
                              phase<=0;
                              cstate<=ST_SEND;
                          end
                      endcase
                  end
                  ST_READ: begin
                      case(phase)
                          0: begin
                              scl_r<=0;
                              sda_drive_low<=0;
                              phase<=1;
                          end
                          1: begin
                              scl_r<=1;
                              phase<=2;
                          end
  						2: begin
                              scl_r<=1;
                              byte_rx[bit_count]<=sda_r2;//SCL高电平的时候采样数据
                              phase<=3;
                          end
                          3: begin
                              scl_r<=0;
                              phase<=0;
                              if(bit_count==0) begin
                                  // 高字节接收完成
                                  if(flow_step==14) begin
                                      mpu_zh_read<=byte_rx;
                                      // 高字节后主机发送ACK
                                      master_ack_low<=1;
                                      flow_step<=15;
                                      cstate<=ST_MASTER_ACK;
                                  end
                                  // 低字节接收完成
                                  else if(flow_step==16) begin
                                      mpu_zl_read<=byte_rx;
                                      // 暂存完整16位数据
                                      pending_raw_data <= {
                                          mpu_zh_read,
                                          byte_rx//不能是mpu_zl_read，因为是非阻塞赋值
                                      };//分两次采集数据，两次采完才合成新数据
                                      pending_sample<=1;
                                      //最后一个字节后主机发送NACK
                                      master_ack_low<=0;
                                      flow_step<=17;
                                      cstate<=ST_MASTER_ACK;
                                  end
                                  else begin
                                      transaction_ok<=0;
                                      pending_sample<=0;
                                      cstate<=ST_STOP;
                                  end
                              end
                              else begin
                                  bit_count<= bit_count-1;
                              end
                          end
                      endcase
                  end
                  ST_MASTER_ACK: begin
                      case(phase)
                          0: begin
                              scl_r<=0;
                              sda_drive_low<=master_ack_low;
                              phase<=1;
                          end
                          1: begin
                              scl_r<=1;
                              sda_drive_low<=master_ack_low;
                              phase<=2;
                          end
                          2: begin
                              scl_r<=1;
                              sda_drive_low<=master_ack_low;
                              phase<=3;
                          end
                          3: begin
                              scl_r<=0;
                              sda_drive_low<=0;
                              phase<=0;
                              // 高字节ACK后继续读取低字节
                              if(flow_step==15) begin
                                  byte_rx<=0;
                                  bit_count<=7;
                                  flow_step<=16;
                                  cstate <= ST_READ;
                              end
                              // 低字节NACK后产生STOP
                              else if(flow_step==17) begin
                                  flow_step<=18;
                                  cstate<=ST_STOP;
                              end
                              else begin
                                  transaction_ok<=0;
                                  pending_sample<=0;
                                  cstate<=ST_STOP;
                              end
                          end
                      endcase
                  end
                  ST_STOP: begin
                      case(phase)
                          0: begin
                              scl_r<=0;
                              sda_drive_low<=1;
                              phase<=1;
                          end
                          1: begin
                              scl_r<=1;
                              sda_drive_low<=1;
                              phase<=2;
                          end
  
                          2'd2: begin
                              scl_r<=1;
                              sda_drive_low<=0;//SDA由低变高,STOP的标志
                              phase <= 2'd3;
                          end
                          2'd3: begin
                              scl_r<= 1;
                              sda_drive_low <= 1'b0;
                              wait_count<=0;
                              phase<=0;
                              cstate<=ST_WAIT;
                          end
                      endcase
                  end
                  ST_WAIT: begin//得在WAIT等10MS才能重新开始
                     scl_r<=1;
                      sda_drive_low<=0;
                      if(wait_count == 12'd3999) begin
                          wait_count <= 12'd0;
                          // 只有事务成功并且SDA已经恢复高电平，
                          // 才认为本次事务真正成功
                          if(transaction_ok && sda_r2) begin
                              // 完成一个初始化寄存器
                              if(times<4)
                                  times <= times+1;
                              // 五个初始化寄存器全部完成
                              else if(times==4) begin
                                  times<=5;
                                  i2c_init_done <= 1'b1;
                              end
                              // 完成一组Z轴高低字节读取
                              else begin
                                  times<=5;
                                  if(pending_sample) begin
                                      raw_data_r <= pending_raw_data;
                                      raw_data_vld_r<=1;
                                      raw_data_vld_count
                                          <= raw_data_vld_count + 1'b1;
                                  end
                              end
                          end
                          pending_sample<=0;
                          // SDA仍为低，重新执行9个恢复时钟
                          if(sda_r2==0) begin
                              recover_count<=0;
                              cstate <= ST_RECOVER;
                          end
                          else begin
                              cstate <= ST_IDLE;
                          end
                      end
                      else begin
                          wait_count <= wait_count + 1'b1;
                      end
                  end
                  default: begin
                      scl_r<=1;
                      sda_drive_low<=0;
                      phase<=0;
                      recover_count<=0;
                      pending_sample<=0;
                      cstate <= ST_RECOVER;
                  end
  
              endcase
          end
      end
  end
  // 原始Z轴数据输出
  assign raw_data = raw_data_r;
    //姿态解算部分
    wire straight_enable=(car_state==4'b0001|| car_state==4'b0010);
    parameter YAW_KI_SHIFT  = 4;    // 积分衰减（右移4位，相当于除以16）
    parameter GYRO_KP_SHIFT = 2;   // 比例衰减（右移2位，相当于除以4）
    parameter ERROR_SPEED   = 5;  // 比例放大系数（修正力度）
    parameter GYRO_DEADBAND = 2;   // 剔除零漂后，用于过滤底盘震动的极小死区
    reg [6:0]calib_cnt;//计算有没有到达64次，到了之后就求零点的平均值
    reg [21:0]calib_sum;//计算初始64位数据的总和，本来一个数据是16位，也就是[15:0],加了64次也就是乘了2的6次方，左移6位
    reg calib_done;//初始零点计算有没有结束
    reg [15:0]zero_offset;//零点的平均值
    wire [21:0]gyro_ext={{6{raw_data[15]}},raw_data};//扩展到22位，方便累加
    reg [15:0]data_r;
    reg [31:0]yaw_integral;
    wire [15:0]true_gyro=raw_data-zero_offset;//真实值=原始值-零漂误差值
    wire true_gyro_sign=true_gyro[15];
    wire [15:0]true_gyro_abs=true_gyro_sign?(~true_gyro+1):true_gyro;
    wire yaw_sign=yaw_integral[31];
    wire [31:0]yaw_abs=yaw_sign?(~yaw_integral+1):yaw_integral;//求补码，如果是负数的话就是取反再加1
    wire gyro_r_sign=data_r[15];
    wire [15:0]gyro_r_abs=gyro_r_sign?(~data_r+1):data_r;
    wire [31:0]yaw_scaled=yaw_abs>>YAW_KI_SHIFT;
    wire [15:0]gyro_scaled=gyro_r_abs>>GYRO_KP_SHIFT;//补码右移
    wire [31:0]yaw_term=yaw_sign?(~yaw_scaled+1):yaw_scaled;
    wire [31:0]gyro_term=gyro_r_sign?({~{16'd0,gyro_scaled}+32'd1}):{16'd0,gyro_scaled};
    wire [31:0]ctrl_out=yaw_term+gyro_term;
    wire ctrl_sign=ctrl_out[31];
    wire [31:0]ctrl_abs=ctrl_sign?(~ctrl_out+1):ctrl_out;
  wire [31:0]ctrl_effective=(ctrl_abs>3)?(ctrl_abs-3):0;
    wire [34:0]calc_error_full={3'd0,ctrl_effective}*ERROR_SPEED;
  wire [31:0]gyro_ext_32={{16{data_r[15]}},data_r};
  wire [31:0]yaw_sum=yaw_integral+gyro_ext_32;

  wire yaw_overflow_pos=(~yaw_integral[31]) && (~gyro_ext_32[31]) && yaw_sum[31];//正数溢出，两个正数相加最后符号位变成了负数
  wire yaw_overflow_neg=(yaw_integral[31]) && (gyro_ext_32[31]) && ~yaw_sum[31];//负数溢出，两个负数相加最后符号位变成了正数
  localparam [31:0] CTRL_DEADBAND = 32'd512;
  wire [31:0] ctrl_level;
  assign ctrl_level = ctrl_abs >> 9;
    always @ ( posedge clk_50m or posedge rst )
    begin //1
    	if(rst)begin//2
        	calib_cnt<=0;
            calib_done<=0;
            calib_sum<=0;
            error_mpu<=0;
            data_r<=0;
            yaw_integral<=0;
            zero_offset<=0;
            mpu_dir<=0;
  
     
        end//2
        else begin //3
        		begin//4
                if(~calib_done && i2c_init_done && raw_data_vld_r)begin
     				if(calib_cnt<64) begin//5
                    calib_cnt<=calib_cnt+1;
                    calib_done<=0;
                    calib_sum<=calib_sum+gyro_ext;//没到64次就一直累加
                    end//5
                    else begin//6
                    zero_offset<=calib_sum[21:6];
                    calib_done<=1;
                    calib_sum<=0;
                    calib_cnt<=0;
                    end//6
                   end
      else begin      // 不处于直行/后退状态时，
      // 清除本次直线控制的航向累计值
      if(calib_done && !straight_enable) begin
          data_r       <= 16'd0;
          yaw_integral <= 32'd0;
          error_mpu    <= 4'd0;
          mpu_dir      <= 1'b0;
      end
     else  if(calib_done && raw_data_vld_r)//自检完成
        begin//7
        			begin if(true_gyro_abs>GYRO_DEADBAND)
                    	data_r<=true_gyro;
                     else data_r<=0;
                    end
                    begin 
  					if(yaw_overflow_pos)
      yaw_integral <= 32'h7FFF_FFFF;  // 最大正数
  else begin if(yaw_overflow_neg)
      yaw_integral <= 32'h8000_0000;  // 最小负数
  else
      yaw_integral <= yaw_sum;
      end
      end
                       if(ctrl_abs < CTRL_DEADBAND) begin
      error_mpu <= 4'd0;
  
      // error_mpu=0时方向没有控制意义
      mpu_dir <= 1'b0;
  end
  else begin
      mpu_dir <= ctrl_sign;
  
      if(ctrl_level > 32'd15)
          error_mpu <= 4'd15;
      else
          error_mpu <= ctrl_level[3:0];
  end
                                    end
                                    end
                                    end
              end  //4  
        end//3
    //1
endmodule    //MPU_IIC_SENSOR
