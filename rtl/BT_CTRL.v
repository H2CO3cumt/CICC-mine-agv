module BT_CTRL(
input wire clk_50m,
input wire rst,
input wire rx,
output reg [8:0] key_cmd,
output wire [7:0] rx_data,
output wire rx_data_vld
);

  //----Code starts here: integrated by Robei-----
  localparam CLOCK=50_000_000;//系统时钟频率
  localparam BAUD=115200;//波特率；
  localparam CHECK_BIT="None";//无校验位
  parameter MAX_1BAUD=CLOCK/BAUD;//一个波特率所要经历的时钟周期
  parameter IDLE=5'b00000;
  parameter START=5'b00001;
  parameter DATA=5'b00010;
  parameter CHECK=5'b00100;
  reg [15:0]cnt_baud;//计算baud有没有到达MAX_1BAUD
  reg [3:0]cnt_bit;//计算bit有没有达到max_bit;
  reg [3:0]max_bit;//bit的最大值，根据不同状态所决定
  reg [4:0]cstate;//当前状态
  reg [4:0]nstate;//下一时钟状态
  reg rx_check;//校验位的实际值
  reg [7:0]rx_temp;//类似于仓库，存储要发送的数据
  reg rx_r1;
  reg rx_r2; 
  wire rx_nege;//用于rx下降沿检测
  wire add_baud;
  wire end_baud;//波特率开始，结束计算
  wire add_bit;
  wire end_bit;//bit开始，结束运算
  wire IDLE_START=(cstate==IDLE) && rx_nege;
  wire START_DATA=(cstate==START)&& end_bit;
  wire DATA_CHECK=(cstate==DATA)&& end_bit && (CHECK_BIT!="None");
  wire DATA_IDLE=(cstate==DATA)&& end_bit && (CHECK_BIT=="None");
  wire CHECK_IDLE=(cstate==CHECK)&& end_bit;
  //wire check_val;//校验位的正确值;经计算得到
  wire add_bit1;
  wire add_bit2;//检测end_baud的上升沿
  always @ ( posedge clk_50m or posedge rst )
  begin 
  	if(rst) begin
      		rx_r1<=1;
          rx_r2<=1;
              end
       else begin
      	    rx_r1<=rx;
          rx_r2<=rx_r1;
      end
  end
  assign rx_nege=~rx_r1 && rx_r2;//检测rx刚刚经历了下降沿  
  
  always @ ( posedge clk_50m or posedge rst )
  begin
  	if(rst)
      		 cnt_baud<=0;
       else if(add_baud)begin
      			if(end_baud)
                  cnt_baud<=0; 
              else cnt_baud<=cnt_baud+1;                
      end
  	else
      		cnt_baud<=0;    
  end
  assign add_baud=cstate!=IDLE;
  assign end_baud=add_baud && (cnt_baud==MAX_1BAUD-1);
  always @ ( * )
  begin 
  	case( cstate)
      IDLE:max_bit=0;
      START:max_bit=1;
      DATA:max_bit=8;
      CHECK:max_bit=1;
          default: max_bit=0   ;
      endcase
  end
  always @ ( posedge clk_50m or posedge rst )
  begin
  	if(rst)
      		 cnt_bit<=0;
       else if(add_bit)begin
      			if(end_bit)
                  cnt_bit<=0; 
              else cnt_bit<=cnt_bit+1;                
      			end
  	else
      		cnt_bit<=cnt_bit;    
  end
  assign add_bit=end_baud;
  assign end_bit=add_bit && (cnt_bit==max_bit-1);
  always @ ( posedge clk_50m or posedge rst )
  begin 
  	if(rst)
      		cstate<=IDLE;
      else cstate<=nstate;
  end
  always @ ( * )
  begin 
  	case( cstate)
      IDLE:if(IDLE_START)
      			nstate=START;
           else nstate=cstate;
      START:if( cnt_baud==MAX_1BAUD>>1 && rx_r2==1)
      			nstate<=IDLE;
     else begin if(START_DATA)
      			nstate=DATA;
            else nstate=cstate;
          end
      	DATA:if(DATA_CHECK)
          		nstate=CHECK;
            else  begin if(DATA_IDLE)
          			nstate=IDLE;
                      else nstate=cstate;
                      end
      	CHECK:if(CHECK_IDLE)
          		nstate=IDLE;
             else nstate=cstate;		   
          default: nstate=IDLE;
      endcase
  end 
  always @ ( posedge clk_50m or posedge rst )
  begin 
  	if(rst)
      		rx_temp<=0;
      else  begin if(cstate==DATA && cnt_baud== MAX_1BAUD>>1)
      rx_temp[cnt_bit]<=rx_r2;
      else rx_temp<=rx_temp;
      end
  end
  assign rx_data=rx_temp;
  always @ (posedge clk_50m or posedge rst  )
  begin 
  	if(rst)
      rx_check<=0;
      else if(cstate==CHECK && cnt_baud==MAX_1BAUD>>1)
      		rx_check<=rx_r2;
  end
  assign rx_data_vld=DATA_IDLE;
  //原来的CMD_PARSER模块
  always @ ( posedge clk_50m or posedge rst)
  begin if(rst)
  	key_cmd<=9'b111_111_111;
      else  begin if(rx_data_vld) begin
  		case( rx_data)
          8'h46 : key_cmd <= 9'b111_111_101; // 'F' GO
          8'h42 : key_cmd <= 9'b101_111_111; // 'B' BACK
          8'h4C : key_cmd <= 9'b111_110_111; // 'L' LEFT
          8'h52 : key_cmd <= 9'b111_011_111; // 'R' RIGHT
          8'h53 : key_cmd <= 9'b111_101_111; // 'S' IDLE
              default: key_cmd<=key_cmd;
          endcase
          end
          else key_cmd<=key_cmd; 
          end
  end
endmodule    //BT_CTRL
