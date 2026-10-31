module CLK_DIV_TOP(
input wire clk_50m,
output reg en_1,
output reg en_1k,
output reg en_10k,
output reg en_1m,
output reg en_500,
output reg en_2k,
output wire rst,
input wire [8:0] key_value,
output reg en_10,
output reg en_100
);

  //----Code starts here: integrated by Robei-----
  parameter MAX_500=1_000_00-1;
  parameter MAX_2k=25_000-1;
  parameter MAX_1=50_000_000-1;
  parameter MAX_1k=50_000-1;
  parameter MAX_10k=5_000-1;
  parameter MAX_1m=50-1;
  parameter MAX_10=5_000_000-1;
  parameter MAX_100=5_000_00-1;
  reg[23:0]cnt_100;
  reg [19:0]cnt_500;
  reg [15:0]cnt_2k;
  reg [27:0]cnt_1;
  reg [15:0]cnt_1k;
  reg [15:0]cnt_10k;
  reg [7:0]cnt_1m;
  reg [23:0] rst_cnt;
  reg [23:0] cnt_10;
  reg rst_r;
  initial begin
  rst_cnt=0;
  rst_r=0;
  end
  
  always @(posedge clk_50m) begin
      if (rst_cnt != 24'd5_000_000) //保持0.1s的高电平
      begin
          rst_cnt <= rst_cnt + 1;
          rst_r <= 1;          // 复位期间保持高
      end else begin
          rst_r <= 0;          // 复位结束后拉低
      end
  end
  assign rst=(key_value==9'b111_111_110)?1:rst_r;
  always @ (posedge clk_50m or posedge rst  )
  begin 
  	if(rst)begin
      cnt_500<=0;
      en_500<=0;
      end 
      else begin
      		if(cnt_500==MAX_500) begin
              cnt_500<=0;
              en_500<=1;
              end
           else begin cnt_500<=cnt_500+1;
          				en_500<=0;
             		end
      end
  end
  always @ (posedge clk_50m or posedge rst  )
  begin 
  	if(rst)begin
      cnt_100<=0;
      en_100<=0;
      end 
      else begin
      		if(cnt_100==MAX_100) begin
              cnt_100<=0;
              en_100<=1;
              end
           else begin cnt_100<=cnt_100+1;
          				en_100<=0;
             		end
      end
  end
  always @ (posedge clk_50m or posedge rst )
  begin 
  	if(rst)begin
      cnt_2k<=0;
      en_2k<=0;
      end 
      else begin
      		if(cnt_2k==MAX_2k) begin
              cnt_2k<=0;
              en_2k<=1;
              end
           else begin cnt_2k<=cnt_2k+1;
          				en_2k<=0;
             		end
      end
  end
  always @ (posedge clk_50m or posedge rst  )
  begin 
  	if(rst)begin
      cnt_1<=0;
      en_1<=0;
      end 
      else begin
      		if(cnt_1==MAX_1) begin
              cnt_1<=0;
              en_1<=1;
              end
           else begin cnt_1<=cnt_1+1;
          				en_1<=0;
             		end
      end
  end
  always @ (posedge clk_50m or posedge rst )
  begin 
  	if(rst)begin
      cnt_1k<=0;
      en_1k<=0;
      end 
      else begin
      		if(cnt_1k==MAX_1k) begin
              cnt_1k<=0;
              en_1k<=1;
              end
           else begin cnt_1k<=cnt_1k+1;
          				en_1k<=0;
             		end
      end
  end
  always @ (posedge clk_50m or posedge rst )
  begin 
  	if(rst)begin
      cnt_10k<=0;
      en_10k<=0;
      end 
      else begin
      		if(cnt_10k==MAX_10k) begin
              cnt_10k<=0;
              en_10k<=1;
              end
           else begin cnt_10k<=cnt_10k+1;
          				en_10k<=0;
             		end
      end
  end
  always @ (posedge clk_50m or posedge rst  )
  begin 
  	if(rst)begin
      cnt_1m<=0;
      en_1m<=0;
      end 
      else begin
      		if(cnt_1m==MAX_1m) begin
              cnt_1m<=0;
              en_1m<=1;
              end
           else begin cnt_1m<=cnt_1m+1;
          				en_1m<=0;
             		end
      end
  end
  always @ (posedge clk_50m or posedge rst )
  begin 
  	if(rst)begin
      cnt_10<=0;
      en_10<=0;
      end 
      else begin
      		if(cnt_10==MAX_10) begin
              cnt_10<=0;
              en_10<=1;
              end
           else begin cnt_10<=cnt_10+1;
          				en_10<=0;
             		end
      end
  end
endmodule    //CLK_DIV_TOP
