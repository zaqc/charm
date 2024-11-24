module full_scan(
	input						rst_n,
	input						sys_clk,
	input						rcv_clk,
	
	input						i_send_sync,	// sys_clk Clock Domain
	
	input						i_rcv_data,
	input						i_rcv_fs,
	
	output		[31:0]			o_st_data,
	output						o_st_vld,
	input						i_st_rdy
);

//	wire		[15:0]			r_data;
//	wire						r_vld;
//
//	rcvr rcvr_u0(
//		.i_clk(rcv_clk),
//		.i_fs(i_rcv_fs),
//		.i_d(i_rcv_data),
//		
//		.o_data(r_data),
//		.o_vld(r_vld)
//	);
	
	wire		[31:0]			adc_data;
	wire						adc_vld;
	wire						st_done;
	wire						crc_ok;

	decode_pkt decode_pkt_u0(
		.rst_n(rst_n),
		.clk(rcv_clk),
		
		.i_rcv_fs(i_rcv_fs),
		.i_rcv_data(i_rcv_data),
		
		.o_st_data(adc_data),
		.o_st_vld(adc_vld),
		
		//.o_st_done(st_done),
		.o_pkt_crc_ok(crc_ok)
	);
		
//	pack_adc pack_adc_u0(
//		.rst_n(rst_n),
//		.clk(rcv_clk),
//		.i_adc_data(r_data[9:0]),
//		.i_adc_vld(r_vld),
//		.o_out_data(adc_data),
//		.o_out_vld(adc_vld)
//	);
	
	reg			[12:0]			wr_ptr;
	reg			[0:0]			wr_half;
	wire						rcv_sync;
	always @ (posedge rcv_clk or negedge rst_n)
		if(~rst_n) begin
			wr_ptr <= 13'd0;
			wr_half <= 1'b0;
		end
		else
			if(rcv_sync) begin
				wr_ptr <= 13'd0;
				wr_half <= ~wr_half;				
			end
			else
				if(adc_vld && ~wr_ptr[12])
					wr_ptr <= wr_ptr + 1'd1;
					
	reg			[12:0]			rd_ptr;
	reg			[0:0]			rd_half;
					
	fs_mem_buf fs_mem_buf_u0(
		.wrclock(rcv_clk),
		.wraddress({wr_half, wr_ptr[11:0]}),
		.data(adc_data),
		.wren(adc_vld && ~wr_ptr[12]),
		
		.rdclock(sys_clk),
		.rdaddress({rd_half, rd_ptr[11:0]}),
		.q(o_st_data)
	);
	
	reg			[1:0]			sync;
	always @ (posedge sys_clk) sync <= {sync[0], i_send_sync};
	
	reg			[0:0]			latch_sync;
	reg			[0:0]			rcv_sync_latch;
	always @ (posedge sys_clk or negedge rst_n)
		if(~rst_n)
			latch_sync <= 1'b0;
		else
			if(sync == 2'b01)
				latch_sync <= 1'b1;
			else
				if(rcv_sync_latch)
					latch_sync <= 1'b0;
					
	always @ (posedge rcv_clk or negedge rst_n)
		if(~rst_n)
			rcv_sync_latch <= 1'b0;
		else
			rcv_sync_latch <= latch_sync;
	
	reg			[0:0]			prev_rcv_sync_latch;
	always @ (posedge rcv_clk) prev_rcv_sync_latch <= rcv_sync_latch;
	
	assign rcv_sync = ~prev_rcv_sync_latch && rcv_sync_latch;
	
	reg			[0:0]			rd_ws;
	always @ (posedge sys_clk or negedge rst_n)
		if(~rst_n) begin
			rd_ptr <= 13'd0;
			rd_half <= 1'b0;
			rd_ws <= 1'b0;
		end
		else
			if(sync == 2'b01) begin
				rd_ptr <= 13'd0;
				rd_half <= wr_half;
				rd_ws <= 1'b0;
			end
			else
				if(~rd_ws)
					rd_ws <= 1'd1;
				else
					if(i_st_rdy && ~rd_ptr[12]) begin
						rd_ptr <= rd_ptr + 1'd1;
						rd_ws <= 1'b0;
					end

	assign o_st_vld = rd_ws & ~rd_ptr[12];
endmodule

