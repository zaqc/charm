module rcvr(
	input						i_fs,
	input						i_clk,
	input						i_d,
	
	input		[31:0]			i_crc,
	
	output		[15:0]			o_data,
	output		[31:0]			o_crc,
	output						o_vld
);

	reg			[31:0]			res_crc;
	wire		[31:0]			out_crc;
	crc32_one_bit crc32_one_bit_u0(
		.crcIn(~|{bit_cntr} ? i_crc : res_crc),
		.crcOut(out_crc),
		.data(i_d)
	);
	
	reg			[4:0]			bit_cntr;
	
	always @ (posedge i_clk)
		if(i_fs)
			bit_cntr <= 5'd0;
		else 
			if(~bit_cntr[4]) begin
				bit_cntr <=  bit_cntr + 1'd1;
				res_crc <= out_crc;
			end
			
	reg			[15:0]			shift_data;
	always @ (posedge i_clk) 
		if(i_fs)
			shift_data <= 16'd0;
		else
			shift_data <= {shift_data[14:0], i_d};
		
	reg			[15:0]			out_data;
	reg			[0:0]			out_vld;
		
	always @ (posedge i_clk)
		if(&{bit_cntr[3:0]}) begin
			out_data <= {shift_data[14:0], i_d};
			out_vld <= 1'b1;
		end
		else
			out_vld <= 1'b0;
	
	assign o_data = out_data;
	assign o_crc = res_crc;
	assign o_vld = out_vld;
endmodule

