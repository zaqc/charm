module rcvr(
	input						rst_n,
	
	input						i_fs,
	input						i_clk,
	input						i_d,
		
	output		[15:0]			o_data,
	output						o_vld
);
	reg			[4:0]			bit_cntr;
		
	always @ (negedge i_clk or negedge rst_n)
		if(~rst_n) 
			bit_cntr <= 5'd0;
		else
			if(~bit_cntr[4])
				bit_cntr <=  bit_cntr + 1'd1;
			else
				if(i_fs)
					bit_cntr <= 5'd1;
				else
					bit_cntr <= 5'd0;

				
	reg			[15:0]			shift_data;
	always @ (negedge i_clk)
		shift_data <= {shift_data[14:0], i_d};
		
	//reg			[31:0]			out_crc;
	reg			[15:0]			out_data;
	reg			[0:0]			out_vld;
	
	always @ (negedge i_clk) begin
		out_vld <= 1'b0;
		if(bit_cntr[4]) begin
			out_data <= {shift_data[14:0], i_d};
			out_vld <= 1'b1;
		end
	end
	
	assign o_data = {shift_data[14:0], i_d};
	assign o_vld = bit_cntr[4];

//	assign o_data = out_data;
//	assign o_vld = out_vld;

endmodule

