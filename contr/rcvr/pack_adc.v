module pack_adc(
	input						rst_n,
	input						clk,
	input		[9:0]			i_adc_data,
	input						i_adc_vld,
	output		[31:0]			o_out_data,
	output						o_out_vld
);

	reg			[1:0]			adc_cnt;
	reg			[31:0]			out_data;
	reg			[0:0]			out_vld;
	always @ (posedge clk or negedge rst_n)
		if(~rst_n) begin
			adc_cnt <= 2'd0;
			out_vld <= 1'b0;
		end
		else begin
			out_vld <= 1'b0;
			out_data <= {out_data[22:0], i_adc_data};
			if(i_adc_vld) begin
				if(adc_cnt == 2'd2) begin
					adc_cnt <= 2'd0;
					out_vld <= 1'b1;
				end
				else
					adc_cnt <= adc_cnt + 1'd1;
			end
		end
		
	assign o_out_data = out_data;
	assign o_out_vld = out_vld;
endmodule
