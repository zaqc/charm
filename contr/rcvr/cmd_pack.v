module cmd_pack(
	input						rst_n,
	input						clk,
	
	input		[31:0]			i_in_data,
	input						i_in_vld,
	output						o_in_rdy,
	
	output						o_fs,
	output						o_d
);

	reg			[2:0]			state;
	
	wire						rdy;
	assign rdy = ~|{state};
	
	reg			[31:0]			data;
	
	wire						tx_vld;
	assign tx_vld = state[2];
	
	wire						tx_rdy;
	
	always @ (posedge clk or negedge rst_n)
		if(~rst_n)
			state <= 3'd0;
		else
			if(rdy) begin
				if(i_in_vld) begin
					data <= i_in_data;
					state <= 3'b100;
				end
			end
			else begin
				if(tx_rdy)
					if(~&{state[1:0]})
						state[1:0] <= state[1:0] + 1'd1;
					else
						state <= 3'd0;
			end
			
	wire		[15:0]			tx_data;
	assign tx_data = 
		state == 3'b100 ? 16'h55FF :
		state == 3'b101 ? data[31:16] :
		state == 3'b110 ? data[15:0] :
		state == 3'b111 ? 16'hFFAA : 16'hXXXX;
		
	tsmt tsmt_u0(
		.rst_n(rst_n),
		.clk(clk),
		
		.i_tx_data(tx_data),
		.i_tx_vld(tx_vld),
		.o_tx_rdy(tx_rdy),
		
		.o_fs(o_fs),
		.o_d(o_d)
	);
	
	assign o_in_rdy = rdy;
	
endmodule

