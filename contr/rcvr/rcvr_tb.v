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
	rcvr rcvr_u0(
		.i_clk(clk),
		.i_fs(cs),
		.i_d(d)
	);

endmodule
