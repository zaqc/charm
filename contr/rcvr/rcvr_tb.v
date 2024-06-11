module data_provider_tb;

	reg			[0:0]			rst_n;
	reg			[0:0]			sys_clk;
	reg			[0:0]			pll_lock;
	
	initial begin
		$display("Start...");
		$display(`TARGET_NAME);
		//$dumpfile("dumpfile_sdrc.vcd");
		$dumpfile({"dumpfile_", `TARGET_NAME, ".vcd"});
		$dumpvars(0);
		pll_lock <= 1'b0;
		rst_n <= 1'b0;
		#10
		rst_n <= 1'b1;
		#10
		pll_lock <= 1'b1;
		#3000000
		$finish();
	end
	
	initial begin
		sys_clk <= 1'b0;
		forever begin
			#5
			sys_clk <= ~sys_clk;
		end
	end
	
	reg			[0:0]			us_clk;
	initial begin
		us_clk <= 1'b0;
		forever begin
			#37
			us_clk <= ~us_clk;
		end
	end
	
	reg			[0:0]			sync;
	initial begin
		sync <= 1'b0;
		#50
		@ (negedge sys_clk) sync <= 1'b1;
		#20
		@ (negedge sys_clk) sync <= 1'b0;
		#140000
		@ (negedge sys_clk) sync <= 1'b1;
		#20
		@ (negedge sys_clk) sync <= 1'b0;
		#140000
		@ (negedge sys_clk) sync <= 1'b1;
		#20
		@ (negedge sys_clk) sync <= 1'b0;
	end
	
	wire						st_vld;
	
	reg			[0:0]			rdy;
	initial begin
		rdy <= 1'b1;
		#10100
		rdy <= 1'b0;
		#1000
		rdy <= 1'b1;
	end

	integer i;	
	reg			[0:0]			cs;
	reg			[0:0]			clk;
	reg			[0:0]			d;
	reg			[15:0]			num;
	initial begin
		cs <= 1'b0;
		clk <= 1'b0;
		num <= 16'd0;
		forever begin
			for(i = 0; i < 16; i = i + 1) begin
				d <= num[15 - i];
				cs <= ~|{i};				
				clk <= 1'b1;
				#10
				clk <= 1'b0;
				#10
				clk <= 1'b0;
			end
			#20
			cs <= 1'b0;
			num <= num + 1'd1;
		end
	end
	
	reg			[15:0]			tst_val;
	wire						tsmt_rdy;
	always @ (posedge sys_clk or negedge rst_n)
		if(~rst_n)
			tst_val <= 16'd0;
		else
			if(tsmt_rdy)
				tst_val <= tst_val + 1'd1;
	
	wire						tsmt_fs;
	wire						tsmt_d;
	tsmt tsmt_u0(
		.rst_n(rst_n),
		.clk(sys_clk),
		.i_tx_data(tst_val),
		.i_tx_vld(1'b1),
		.o_tx_rdy(tsmt_rdy),
		
		.o_fs(tsmt_fs),
		.o_d(tsmt_d)
	);
	
	rcvr rcvr_u0(
		.i_clk(sys_clk),
		.i_fs(tsmt_fs),
		.i_d(tsmt_d)
	);
	
	full_scan full_scan_u0(
		.rst_n(rst_n),
		.sys_clk(sys_clk),
		.rcv_clk(clk),
		.i_rcv_fs(cs),
		.i_rcv_data(d),
		
		.i_send_sync(sync),
		
		.i_st_rdy(1'b1)
	);
	
	reg			[31:0]			pack_cntr;
	always @ (posedge sys_clk or negedge rst_n)
		if(~rst_n)
			pack_cntr <= 32'd0;
		else
			pack_cntr <= pack_cntr + 32'h12345678;
	
	cmd_pack cmd_pack_u0(
		.rst_n(rst_n),
		.clk(sys_clk),
		
		.i_in_data(pack_cntr),
		.i_in_vld(1'b1)
	);

endmodule
