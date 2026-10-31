module KEY_DEBOUNCE(
input wire clk_50m,
input wire rst,
output reg [8:0] key_value,
input wire [8:0] key_in,
input wire en_10k
);

  //----Code starts here: integrated by Robei-----
  //接入10KHZ的时钟，能有效应对矿物运输过程中的干扰因素
  //key_value的值对应如下：
  //111_111_110->1
  //111_111_101->2
  //111_111_011->3
  //111_110_111->4
  //111_101_111->5
  //111_011_111->6
  //110_111_111->7
  //101_111_111->8
  //011_111_111->9
  reg [8:0] key1;//三次的采样值
  reg [8:0] key2;
  reg [8:0] key3;
  reg [8:0]key_old;
  reg key_trig;
  always @ ( posedge clk_50m or posedge rst )
  begin if(rst) begin
  		key_value<=9'b111_111_111;
  		key1<=9'b111_111_111;
          key2<=9'b111_111_111;
          key3<=9'b111_111_111;
          key_old<=9'b111_111_111;
          key_trig<=0;
          end
          else begin if(en_10k) begin
          		key3<=key2;
              key2<=key1;
              key1<=key_in;
              end
              if((key1==key2) && (key1==key3)) begin //连续三次采样都相等
              		if(key_old==9'b111_111_111 && key1!=9'b111_111_111)//旧key值为空也就是9'b111_111_111,并且最新值不为空时进入赋值
                      //目的是为了按一次键即时不松手，赋值也只触发一次
                      begin
                      key_trig<=1;
                      key_old<=key3;
                      end
                      else  begin  key_trig<=0;
                      				key_old<=key3;
                      end
  
                      if(key_trig)
              begin
              		case(key3)//按行分类
                     9'b111_111_110:key_value<=key3;//用三次采样后的数据，能尽量避免偶发干扰和短时间抖动（机械抖动和运输干扰）
                  	  9'b111_111_101:key_value<=key3;
                      9'b111_111_011:key_value<=key3;
                      9'b111_110_111:key_value<=key3;
                      9'b111_101_111:key_value<=key3;
                      9'b111_011_111:key_value<=key3;
                      9'b110_111_111:key_value<=key3;
                      9'b101_111_111:key_value<=key3;
                      9'b011_111_111:key_value<=key3;
                      default:key_value<=key_value;
                      endcase
              end
              else key_value<=key_value;//采样失败，保持原值            
          end        
          else begin
          key_old<=key_old;
          key_trig<=key_trig;//消抖不稳定时保持原态     
          end       
          end
  end
  
endmodule    //KEY_DEBOUNCE
