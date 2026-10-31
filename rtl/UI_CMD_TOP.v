module UI_CMD_TOP(
input wire en_10k,
input wire en_1,
input wire rst,
output wire [8:0] key_value,
output wire [1:0] back,
output wire [1:0] go,
output wire [1:0] right,
output wire [5:0] sel,
output wire [7:0] seg,
input wire [3:0] car_state,
output wire buzz_out,
input wire [8:0] key_in,
input wire en_2k,
input wire en_500,
input wire clk_50m,
input wire gas_alarm,
output wire [1:0] left
);

  //----Code starts here: integrated by Robei-----
  
//---Module instantiation---
  KEY_DEBOUNCE KEY_DEBOUNCE4(
    .key_value(key_value),
    .key_in(key_in),
    .clk_50m(clk_50m),
    .en_10k(en_10k),
    .rst(rst)
);

  SEG_LED_BUZZ SEG_LED_BUZZ2(
    .clk_50m(clk_50m),
    .en_500(en_500),
    .rst(rst),
    .car_state(car_state),
    .sel(sel),
    .seg(seg),
    .gas_alarm(gas_alarm),
    .en_2k(en_2k),
    .en_1(en_1),
    .go(go),
    .right(right),
    .left(left),
    .back(back),
    .buzz_out(buzz_out)
);

endmodule    //UI_CMD_TOP
