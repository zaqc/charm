module tsmt(
	input						rst_n,
	input						clk,
	
	input		[15:0]			i_tx_data,
	input						i_tx_vld,
	output						o_tx_rdy,
	
	output						o_fs,
	output						o_d,
	output						o_clk
);

	reg			[3:0]			bit_cntr;
	reg			[15:0]			shift_data;
	reg			[0:0]			send_dummy_bit;
	
	reg			[0:0]			clk_out;
	reg			[0:0]			tx_clk;
	
	always @ (posedge clk or negedge rst_n)
		if(~rst_n)
			tx_clk <= 1'b0;
		else
			if(clk_out)
				tx_clk <= ~tx_clk;
			else
				tx_clk <= 1'b0;
				
	reg			[15:0]			latch_data;
	reg			[0:0]			data_latched;
	reg			[0:0]			data_loaded;
	always @ (posedge clk or negedge rst_n)
		if(~rst_n) 
			data_latched <= 1'b0;
		else
			if(~data_latched) begin
				if(i_tx_vld) begin
					data_latched <= 1'b1;
					latch_data <= i_tx_data;
				end
			end
			else
				if(data_loaded)
					data_latched <= 1'b0;
					
	reg			[0:0]			fs;
	reg			[15:0]			next_data;
	
	always @ (posedge clk or negedge rst_n)
		if(~rst_n) begin
			bit_cntr <= 4'h0;
			send_dummy_bit <= 1'b1;
			clk_out <= 1'b0;
			fs <= 1'b0;
			data_loaded <= 1'b0;
		end
		else begin
			data_loaded <= 1'b0;
			if(~clk_out) begin
				if(data_latched) begin
					bit_cntr <= 4'd0;
					shift_data <= latch_data;
					clk_out <= 1'b1;
					data_loaded <= 1'b1;
				end
			end
			else
				if(~tx_clk) begin
					fs <= 1'b0;
					if(send_dummy_bit) begin
						if(~fs)
							fs <= 1'b1;
						else
							send_dummy_bit <= 1'b0;
					end
					else begin
						shift_data <= {shift_data[14:0], 1'b0};
						if(~&{bit_cntr}) begin
							bit_cntr <= bit_cntr + 1'd1;
							if(bit_cntr == 4'd14)
								if(data_latched) begin
									fs <= 1'b1;
									next_data <= latch_data;
									data_loaded <= 1'b1;
								end
								else begin
									send_dummy_bit <= 1'b1;
									clk_out <= 1'b0;
								end
						end
						else
							if(fs) begin
								bit_cntr <= 4'd0;
								shift_data <= next_data;
							end 
					end
				end
		end
		
	assign o_d = shift_data[15];
	assign o_fs = fs;
	assign o_tx_rdy = ~data_latched;
	
	assign o_clk = tx_clk; //clk_out ? clk : 1'b0;
endmodule

