module tsmt(
	input						rst_n,
	input						clk,
	
	input		[15:0]			i_tx_data,
	input						i_tx_vld,
	output						o_tx_rdy,
	
	output						o_fs,
	output						o_d
);

	reg			[0:0]			busy;
	reg			[0:0]			fs;
	reg			[3:0]			bit_cntr;
	reg			[15:0]			shift_data;
	
	always @ (posedge clk or negedge rst_n)
		if(~rst_n) begin
			busy <= 1'b0;
			bit_cntr <= 4'd0;
			fs <= 1'b0;
		end
		else begin
			fs <= 1'b0;
			if(~busy) begin
				if(i_tx_vld) begin
					bit_cntr <= 4'd0;
					shift_data <= i_tx_data;
					busy <= 1'b1;
					fs <= 1'b1;
				end
			end
			else begin
				if(~fs)
					shift_data <= {shift_data[14:0], 1'b0};
				if(~&{bit_cntr})
					bit_cntr <= bit_cntr + 1'd1;
				else
					busy <= 1'b0;
			end
		end
			
	assign o_d = shift_data[15];
	assign o_fs = fs;
	assign o_tx_rdy = ~busy;
endmodule
