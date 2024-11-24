module Top(
    input clk,
    input rst,
    input echo,
    input left_signal,
    input right_signal,
    input mid_signal,
    output trig,
    output left_motor,
    output reg [1:0]left,
    output right_motor,
    output reg [1:0]right
);
    parameter STAY=3'b000;
    parameter ST2=3'b010;
    parameter ST3=3'b101;
    parameter RIGHTF=3'b001;
    parameter RIGHT=3'b011;
    parameter LEFT=3'b110;
    parameter LEFTF=3'b100;
    parameter straight=3'b111;
    
    parameter FORWARD = 2'b00;
    parameter LEFT_OUT = 2'b01;
    parameter RIGHT_OUT = 2'b10;
    
    wire Rst_n, rst_pb, stop;
    wire [2:0] mode;
    wire [1:0] state;
    wire [1:0] motor_pwm;
    debounce d0(rst_pb, rst, clk);
    onepulse d1(rst_pb, clk, Rst_n);

    motor A(
        .clk(clk),
        .rst(rst),
        .mode(mode),
        .pwm(motor_pwm)
    );
    
    sonic_top B(
        .clk(clk), 
        .rst(rst), 
        .Echo(echo), 
        .Trig(trig),
        .stop(stop)
    );
    
    tracker_sensor C(
        .clk(clk), 
        .reset(rst), 
        .left_signal(left_signal), 
        .right_signal(right_signal),
        .mid_signal(mid_signal),
        .state(state)
       );
    always @(*) begin
        // [TO-DO] Use left and right to set your pwm
        if(stop) {left, right} = {2'b00, 2'b00};
        else  begin
            case (motor_pwm)
                2'b11: {left, right} = {2'b10, 2'b10};
                2'b10: {left, right} = {2'b10, 2'b00};
                2'b01: {left, right} = {2'b00, 2'b10};
                2'b00: {left, right} = {2'b00, 2'b00};
            endcase
        end
    end
    
    assign left_motor = stop ? 1'b0 : 1'b1;
    assign right_motor = stop ? 1'b0 : 1'b1;
    
    assign mode = (state == straight) ? FORWARD : (state == LEFT_OUT) ? RIGHT : (state == RIGHT_OUT) ? LEFT : FORWARD;
    
endmodule

module debounce (pb_debounced, pb, clk);
    output pb_debounced; 
    input pb;
    input clk;
    reg [4:0] DFF;
    
    always @(posedge clk) begin
        DFF[4:1] <= DFF[3:0];
        DFF[0] <= pb; 
    end
    assign pb_debounced = (&(DFF)); 
endmodule

module onepulse (PB_debounced, clk, PB_one_pulse);
    input PB_debounced;
    input clk;
    output reg PB_one_pulse;
    reg PB_debounced_delay;

    always @(posedge clk) begin
        PB_one_pulse <= PB_debounced & (! PB_debounced_delay);
        PB_debounced_delay <= PB_debounced;
    end 
endmodule
