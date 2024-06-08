module rcvr(
	input						i_fs,
	input						i_clk,
	input						i_d,
	
	output		[15:0]			o_data,
	output						o_vld
);

	reg			[3:0]			bit_cntr;

	always @ (posedge i_clk)
		if(i_fs)
			bit_cntr <= 4'd0;
		else
			bit_cntr <= bit_cntr + 1'd1;
			
	reg			[15:0]			shift_data;
	always @ (posedge i_clk)
		if(i_fs)
			shift_data <= 16'd0;
		else
			shift_data <= {shift_data[14:0], i_d};
		
	reg			[15:0]			out_data;
	reg			[0:0]			out_vld;
	
	reg			[0:0]			prev_fs;
	always @ (posedge i_clk) prev_fs <= i_fs;
	
	always @ (posedge i_clk)
		if(bit_cntr == 4'd15) begin
			out_data <= {shift_data[14:0], i_d};
			out_vld <= 1'b1;
		end
		else
			out_vld <= 1'b0;
	
	assign o_data = out_data;
	assign o_vld = out_vld;
endmodule

