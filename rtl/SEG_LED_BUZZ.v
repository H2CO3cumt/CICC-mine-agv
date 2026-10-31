module SEG_LED_BUZZ(
input wire clk_50m,
input wire en_500,
input wire rst,
input wire [3:0] car_state,
output reg [5:0] sel,
output reg [7:0] seg,
input wire gas_alarm,
input wire en_2k,
input wire en_1,
output reg [1:0] go,
output reg [1:0] right,
output reg [1:0] left,
output reg [1:0] back,
output reg buzz_out
);

  //----Code starts here: integrated by Robei-----
  //因为模块限额，我只能把原先的SEL_SEG,LED,BUZZ模块融合，请谅解
  //SEL_SEG部分
    //   // 数字段码
      parameter  SEG_0    = 8'hC0;  // 数字 0
      parameter  SEG_1    = 8'hF9;  // 数字 1
      parameter  SEG_2    = 8'hA4;  // 数字 2
      parameter  SEG_3    = 8'hB0;  // 数字 3
      parameter  SEG_4    = 8'h99;  // 数字 4
      parameter  SEG_5    = 8'h92;  // 数字 5
      parameter  SEG_6    = 8'h82;  // 数字 6
      parameter  SEG_7    = 8'hF8;  // 数字 7
      parameter  SEG_8    = 8'h80;  // 数字 8
      parameter  SEG_9    = 8'h90;  // 数字 9
      // 熄灭码 (全灭)
      parameter  SEG_OFF  = 8'hFF;  // 数码管全灭
      parameter IDLE=4'b0000;
      parameter GO=4'b0001;
      parameter BACK=4'b0010;
      parameter LEFT=4'b0100;
      parameter RIGHT=4'b1000;
      always @ ( posedge clk_50m or posedge rst )
      begin 
      	if(rst)
          		sel<=6'b111_110;
           else if(en_500)
           sel<={sel[4:0],sel[5]};
      end
      always @ ( * )
  	 begin
          	case( car_state)
          IDLE:begin
          				case(sel)
                      6'b111_110:seg=SEG_OFF;
                      6'b111_101:seg=SEG_0;
                      6'b111_011:seg=SEG_0;
                      6'b110_111:seg=SEG_0;
                      6'b101_111:seg=SEG_0;
                      6'b011_111:seg=SEG_OFF;
              			default:seg=SEG_OFF;
          				endcase
          			end
          
          GO:begin
          				case(sel)
                      6'b111_110:seg=SEG_OFF;
                      6'b111_101:seg=SEG_0;
                      6'b111_011:seg=SEG_0;
                      6'b110_111:seg=SEG_0;
                      6'b101_111:seg=SEG_1;
                      6'b011_111:seg=SEG_OFF;
              			default:seg=SEG_OFF;
          				endcase
          			end
          BACK:begin
          				case(sel)
                      6'b111_110:seg=SEG_OFF;
                      6'b111_101:seg=SEG_0;
                      6'b111_011:seg=SEG_0;
                      6'b110_111:seg=SEG_1;
                      6'b101_111:seg=SEG_0;
                      6'b011_111:seg=SEG_OFF;
              			default:seg=SEG_OFF;
          				endcase
          			end
          LEFT:begin
          				case(sel)
                      6'b111_110:seg=SEG_OFF;
                      6'b111_101:seg=SEG_0;
                      6'b111_011:seg=SEG_1;
                      6'b110_111:seg=SEG_0;
                      6'b101_111:seg=SEG_0;
                      6'b011_111:seg=SEG_OFF;
              			default:seg=SEG_OFF;
          				endcase
          			end
          RIGHT:begin
          				case(sel)
                      6'b111_110:seg=SEG_OFF;
                      6'b111_101:seg=SEG_1;
                      6'b111_011:seg=SEG_0;
                      6'b110_111:seg=SEG_0;
                      6'b101_111:seg=SEG_0;
                      6'b011_111:seg=SEG_OFF;
              			default:seg=SEG_OFF;
          				endcase
          			end   
                  default:seg=SEG_6;
              endcase
  
          end 
  //BUZZ部分
  always@(posedge clk_50m or posedge rst)begin
  if(rst)
  buzz_out<=0;
  else begin	if(en_2k) begin
  				if(car_state==4'b0100||car_state==4'b1000||car_state==4'b0010||gas_alarm) 
      					buzz_out<=~buzz_out;
                     else buzz_out<=0;
                          end
                  end
  
  end
   //LED部分
  always @ ( posedge clk_50m or posedge rst )
    begin 
     if(rst) begin
      go<=2'b00;
     back<=2'b00;
     right<=2'b00;
        left<=2'b00;
        end
        else begin if(en_1)
        case( car_state)
        4'b0000:begin
     go<=2'b11;
     back<=2'b11;
     right<=2'b11;
        left<=2'b11;
        end
        4'b0001:begin 
    go<=~go;
     back<=2'b00;
     right<=2'b00;
        left<=2'b00;
        end
        4'b0010:begin 
    go<=2'b00;
     back<=~back;
     right<=2'b00;
        left<=2'b00;
        end
        4'b0100:begin 
    go<=2'b00;
     back<=2'b00;
     right<=2'b00;
        left<=~left;
        end
        4'b1000:begin 
    go<=2'b00;
     back<=2'b00;
     right<=~right;
        left<=2'b00;
    
        end
            default:  begin
            go<=2'b00;
     back<=2'b00;
     right<=2'b00;
        left<=2'b00;
        end 
        endcase
        
        end    
    end
  
endmodule    //SEG_LED_BUZZ
