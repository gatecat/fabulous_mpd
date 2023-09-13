`define OPENFRAME_IO_PADS 44

module mpd_fpga (
    `ifdef USE_POWER_PINS
	inout VPWR,		/* 1.8V domain */
	inout VGND,
    `endif
    input  porb,		/* Power-on-reset (inverted)	 */
    input  por,			/* Power-on-reset (non-inverted) */
    input  resetb,		/* Master (pin) reset (inverted) */
    input  [`OPENFRAME_IO_PADS-1:0] gpio_in,	/* Input from GPIO */
    output [`OPENFRAME_IO_PADS-1:0] gpio_out,	/* Output to GPIO */
    output [`OPENFRAME_IO_PADS-1:0] gpio_oeb,	/* GPIO output enable (inverted) */
    output [`OPENFRAME_IO_PADS-1:0] gpio_ieb,	/* GPIO input enable (inverted) */
    output [`OPENFRAME_IO_PADS-1:0] gpio_ib_mode_sel,	/* GPIO mode */
    output [`OPENFRAME_IO_PADS-1:0] gpio_vtrip_sel,	/* GPIO threshold */
    output [`OPENFRAME_IO_PADS-1:0] gpio_slow_sel,	/* GPIO slew rate */
    output [`OPENFRAME_IO_PADS-1:0] gpio_dm2,		/* GPIO digital mode */
    output [`OPENFRAME_IO_PADS-1:0] gpio_dm1,		/* GPIO digital mode */
    output [`OPENFRAME_IO_PADS-1:0] gpio_dm0,		/* GPIO digital mode */
    input  [`OPENFRAME_IO_PADS-1:0] gpio_loopback_one,	/* Value 1 for loopback */
    input  [`OPENFRAME_IO_PADS-1:0] gpio_loopback_zero	/* Value 0 for loopback */
);

    wire [`OPENFRAME_IO_PADS-1:0] sys_gpio_in;
    wire [`OPENFRAME_IO_PADS-1:0] sys_gpio_out;
    wire [`OPENFRAME_IO_PADS-1:0] sys_gpio_oeb;
    wire [`OPENFRAME_IO_PADS-1:0] sys_gpio_ieb;

    wire [`OPENFRAME_IO_PADS*12-1:0] fabric_config;

    wire fabric_clk = sys_gpio_in[38];
    wire user_clk_i = sys_gpio_in[39];
    wire fab_usr_clk_o;
    wire [1:0] user_clk_sel = sys_gpio_in[41:40];

    wire cfg_uart_rx = sys_gpio_in[43], cfg_sclk = sys_gpio_in[2], cfg_sdata = sys_gpio_in[3], cfg_rxled;

    wire done_led, heart_led;

    wire user_clk = user_clk_sel[1] ? (user_clk_sel[0] ? fab_usr_clk_o : user_clk_i) : fabric_clk;

    // Magic sequence to assert fabric configuration done (these latches should be written last)
    assign done_led = ({D_config_west_C[23:0], C_config_west_C[23:0]} == 48'hFEEDBADCA77E);

    wire done_override = sys_gpio_in[1];
    wire fabric_done = done_override | done_led; // TODO: add done signal from config logic ?

    // Config latches for various purposes
    wire [71:0] A_config_east_C, A_config_west_C,  B_config_east_C,  B_config_west_C;
    wire [71:0] C_config_east_C, C_config_west_C,  D_config_east_C,  D_config_west_C;

    // Fabric external IO
    wire [11:0] fab_I_top, fab_O_top, fab_T_top;
    // Fabric user project IO
    wire [159:0] UIO_BOT_UIN, UIO_BOT_UOUT, UIO_TOP_UIN, UIO_TOP_UOUT;
    // User project external IO
    wire [17:0] uprj_I, uprj_O, uprj_T;

    // heartbeat (so we know clock is alive)
    reg [21:0] beat_ctr;
    always @(posedge fabric_clk, negedge resetb) begin
        if (!resetb)
            beat_ctr <= 21'b0;
        else
            beat_ctr <= beat_ctr + 1'b1;
    end
    assign heart_led = beat_ctr[21];

    eFPGA_top fpga_i (
        .A_config_east_C(A_config_east_C), .A_config_west_C(A_config_west_C),
        .B_config_east_C(B_config_east_C), .B_config_west_C(B_config_west_C),
        .C_config_east_C(C_config_east_C), .C_config_west_C(C_config_west_C),
        .D_config_east_C(D_config_east_C), .D_config_west_C(D_config_west_C),
        .I_top(fab_I_top), .O_top(fab_O_top), .T_top(fab_T_top),
        .UIO_BOT_UIN(UIO_BOT_UIN), .UIO_BOT_UOUT(UIO_BOT_UOUT), .UIO_TOP_UIN(UIO_TOP_UIN), .UIO_TOP_UOUT(UIO_TOP_UOUT),
        //Config related ports
        .CLK(fabric_clk), .resetn(resetb),
        .SelfWriteStrobe(1'b0), .SelfWriteData(32'b0),
        .Rx(cfg_uart_rx), .ComActive(), .ReceiveLED(cfg_rxled),
        .s_clk(cfg_sclk), .s_data(cfg_sdata)
    );

    // non fabric configured IO
    assign fabric_config[7*12-1:0] = 1'b0;
    assign fabric_config[`OPENFRAME_IO_PADS*12-1:37*12] = 1'b0;

    // fabric configured IO
    generate
        genvar ii;
        begin : io_config_assign
            // 18 user IO start from 7, controlled from east side
            for (ii = 0; ii < 18; ii = ii + 1) begin
                assign fabric_config[(ii + 7) * 12 +: 12] =
                    ((ii % 4) == 3) ? {6'b0, D_config_east_C[(ii / 4) * 12 +: 6]} :
                    ((ii % 4) == 2) ? {6'b0, C_config_east_C[(ii / 4) * 12 +: 6]} :
                    ((ii % 4) == 1) ? {6'b0, B_config_east_C[(ii / 4) * 12 +: 6]} :
                                      {6'b0, A_config_east_C[(ii / 4) * 12 +: 6]};
            end
            // 12 fabric IO start from 25, controlled from west side
            for (ii = 0; ii < 12; ii = ii + 1) begin
                assign fabric_config[(ii + 25) * 12 +: 12] = (ii % 2) ? B_config_west_C[(ii / 2) * 12 +: 12] : A_config_west_C[(ii / 2) * 12 +: 12];
            end
        end
    endgenerate

    // GPIO default configurations for each pad
    // bit 12   configurable from fabric
    // bit 11   fixed output value
    // bit 10   fixed output enable value
    // bit 9    fixed input enable value
    // bit 8    override output
    // bit 7    override output enable
    // bit 6    override input enable
    // bit 5    slow slew
    // bit 4    TTL trip point
    // bit 3    I-B mode
    // bits 2-0 digital mode

    // b a 9 8 7 6 5 4 3 2 1 0
    //-------------------------
    // 0 0 1 0 1 1 0 0 0 1 1 0  (13'h02c6) vector mode output
    // 0 1 0 0 1 1 0 0 0 0 0 1  (13'h04c1) vector mode input
    // 1 0 0 1 1 1 0 0 0 0 1 0  (13'h09c2) vector mode input, weak pull-up
    // 0 0 0 1 1 1 0 0 0 0 1 1  (13'h01c3) vector mode input, weak pull-down
    // 0 0 0 0 0 0 0 0 0 1 1 0  (13'h0006) bidirectional controlled mode
    // 0 0 1 1 1 0 0 0 0 1 1 0  (13'h0386) force zero output
    // 1 0 1 1 1 0 0 0 0 1 1 0  (13'h0b86) force one output


    parameter [(`OPENFRAME_IO_PADS*13)-1:0] CONFIG_GPIO_INIT = {
        {13'h04c1},  // GPIO[43] UART RX         input
        {13'h04c1},  // GPIO[42] unused          input
        {13'h04c1},  // GPIO[41] Clock sel 1     input
        {13'h04c1},  // GPIO[40] Clock sel 0     input
        {13'h04c1},  // GPIO[39] User clock      input
        {13'h04c1},  // GPIO[38] Fabric clock    input
        {13'h04c1},  // GPIO[37] unused          input
        {13'h14c1},  // GPIO[36] fabric IO
        {13'h14c1},  // GPIO[35] fabric IO
        {13'h14c1},  // GPIO[34] fabric IO
        {13'h14c1},  // GPIO[33] fabric IO
        {13'h14c1},  // GPIO[32] fabric IO
        {13'h14c1},  // GPIO[31] fabric IO
        {13'h14c1},  // GPIO[30] fabric IO
        {13'h14c1},  // GPIO[29] fabric IO
        {13'h14c1},  // GPIO[28] fabric IO
        {13'h14c1},  // GPIO[27] fabric IO
        {13'h14c1},  // GPIO[26] fabric IO
        {13'h14c1},  // GPIO[25] fabric IO
        {13'h1006},  // GPIO[24] user IO
        {13'h1006},  // GPIO[23] user IO
        {13'h1006},  // GPIO[22] user IO
        {13'h1006},  // GPIO[21] user IO
        {13'h1006},  // GPIO[20] user IO
        {13'h1006},  // GPIO[19] user IO
        {13'h1006},  // GPIO[18] user IO
        {13'h1006},  // GPIO[17] user IO
        {13'h1006},  // GPIO[16] user IO
        {13'h1006},  // GPIO[15] user IO
        {13'h1006},  // GPIO[14] user IO
        {13'h1006},  // GPIO[13] user IO
        {13'h1006},  // GPIO[12] user IO
        {13'h1006},  // GPIO[11] user IO
        {13'h1006},  // GPIO[10] user IO
        {13'h1006},  // GPIO[9] user IO
        {13'h1006},  // GPIO[8] user IO
        {13'h1006},  // GPIO[7] user IO
        {13'h02c6},  // GPIO[6] DONE output
        {13'h02c6},  // GPIO[5] heartbeat LED output
        {13'h02c6},  // GPIO[4] receive LED output
        {13'h04c1},  // GPIO[3] bitbang SCLK
        {13'h04c1},  // GPIO[2] bitbang SDATA
        {13'h04c1},  // GPIO[1] DONE override input
        {13'h04c1}   // GPIO[0] unused
    };

    genvar i;
    generate
    for (i = 0; i < `OPENFRAME_IO_PADS; i = i + 1) begin
        mpd_io_ctrl #(
            .GPIO_DEFAULTS(CONFIG_GPIO_INIT[((i+1)*13)-1:i*13])
        ) mpd_io_ctrl (
            .fabric_done(fabric_done),
            .fabric_config(fabric_config[((i+1)*12)-1:i*12]),

            .sys_gpio_in(sys_gpio_in[i]),
            .sys_gpio_out(sys_gpio_out[i]),
            .sys_gpio_oeb(sys_gpio_oeb[i]),
            .sys_gpio_ieb(sys_gpio_ieb[i]),

            .pad_gpio_in(gpio_in[i]),
            .pad_gpio_out(gpio_out[i]),
            .pad_gpio_oeb(gpio_oeb[i]),
            .pad_gpio_ieb(gpio_ieb[i]),

            .pad_gpio_slow_sel(gpio_slow_sel[i]),
            .pad_gpio_vtrip_sel(gpio_vtrip_sel[i]),
            .pad_gpio_ib_mode_sel(gpio_ib_mode_sel[i]),
            .pad_gpio_dm({gpio_dm2[i], gpio_dm1[i], gpio_dm0[i]})
        );

    end
    endgenerate
    // TODO: integrate user projects...



    assign fab_usr_clk_o = UIO_TOP_UOUT[0];
    wire [1:0] prj_sel = UIO_TOP_UOUT[3:2];
    wire user_reset = UIO_TOP_UOUT[1];

    wire [128*4-1:0] uprj_hpc_out;
    assign UIO_TOP_UIN[127:0] = uprj_hpc_out[prj_sel[1:0] * 128 +: 128];
    assign UIO_TOP_UIN[159:128] = 1'b0;

    // user projects

    RISCV_core RISCV_core(
        .clk(user_clk), .reset(user_reset),
       .o_rftop_rd1(UIO_TOP_UOUT[63:32]),
       .o_rfbot_rd2(UIO_TOP_UOUT[95:64]),
       .i_ROM_instruction(UIO_TOP_UOUT[127:96]),
       .i_dmem_read_data(UIO_TOP_UOUT[159:128]),
       .o_rftop_rs1(uprj_hpc_out[7:0]),
       .o_rfbot_rs2(uprj_hpc_out[15:8]),
       .o_rf_we(uprj_hpc_out[16]),
       .o_rf_wa(uprj_hpc_out[24:17]),
       .o_rf_wd(uprj_hpc_out[56:25]),
       .o_ROM_addr(uprj_hpc_out[64:57]),
       .o_dmem_addr(uprj_hpc_out[74:65]),
       .o_dmem_write_data(uprj_hpc_out[106:75]),
       .o_dmem_write_enable(uprj_hpc_out[110:107])
    );
    assign uprj_hpc_out[127:111] = 1'b0;

    SLICE slice_i (
        .APPLY_INIT(UIO_TOP_UOUT[32]),
        .H_I(UIO_TOP_UOUT[33]),
        .H6(UIO_TOP_UOUT[34]),
        .H5(UIO_TOP_UOUT[35]),
        .H4(UIO_TOP_UOUT[36]),
        .H3(UIO_TOP_UOUT[37]),
        .H2(UIO_TOP_UOUT[38]),
        .H1(UIO_TOP_UOUT[39]),
        .CKEN_B4(UIO_TOP_UOUT[40]),
        .CKEN_B3(UIO_TOP_UOUT[41]),
        .G_I(UIO_TOP_UOUT[42]),
        .G6(UIO_TOP_UOUT[43]),
        .G5(UIO_TOP_UOUT[44]),
        .G4(UIO_TOP_UOUT[45]),
        .G3(UIO_TOP_UOUT[46]),
        .G2(UIO_TOP_UOUT[47]),
        .G1(UIO_TOP_UOUT[48]),
        .SRST_B2(UIO_TOP_UOUT[49]),
        .F_I(UIO_TOP_UOUT[50]),
        .F6(UIO_TOP_UOUT[51]),
        .F5(UIO_TOP_UOUT[52]),
        .F4(UIO_TOP_UOUT[53]),
        .F3(UIO_TOP_UOUT[54]),
        .F2(UIO_TOP_UOUT[55]),
        .F1(UIO_TOP_UOUT[56]),
        .CLK_B2(user_clk),
        .E_I(UIO_TOP_UOUT[58]),
        .E6(UIO_TOP_UOUT[59]),
        .E5(UIO_TOP_UOUT[60]),
        .E4(UIO_TOP_UOUT[61]),
        .E3(UIO_TOP_UOUT[62]),
        .E2(UIO_TOP_UOUT[63]),
        .E1(UIO_TOP_UOUT[64]),
        .HX(UIO_TOP_UOUT[65]),
        .GX(UIO_TOP_UOUT[66]),
        .FX(UIO_TOP_UOUT[67]),
        .EX(UIO_TOP_UOUT[68]),
        .CIN(UIO_TOP_UOUT[69]),
        .AX(UIO_TOP_UOUT[70]),
        .BX(UIO_TOP_UOUT[71]),
        .CX(UIO_TOP_UOUT[72]),
        .DX(UIO_TOP_UOUT[73]),
        .D_I(UIO_TOP_UOUT[74]),
        .D6(UIO_TOP_UOUT[75]),
        .D5(UIO_TOP_UOUT[76]),
        .D4(UIO_TOP_UOUT[77]),
        .D3(UIO_TOP_UOUT[78]),
        .D2(UIO_TOP_UOUT[79]),
        .D1(UIO_TOP_UOUT[80]),
        .SRST_B1(UIO_TOP_UOUT[81]),
        .CKEN_B1(UIO_TOP_UOUT[82]),
        .CKEN_B2(UIO_TOP_UOUT[83]),
        .C_I(UIO_TOP_UOUT[84]),
        .C6(UIO_TOP_UOUT[85]),
        .C5(UIO_TOP_UOUT[86]),
        .C4(UIO_TOP_UOUT[87]),
        .C3(UIO_TOP_UOUT[88]),
        .C2(UIO_TOP_UOUT[89]),
        .C1(UIO_TOP_UOUT[90]),
        .CLK_B1(user_clk),
        .B_I(UIO_TOP_UOUT[92]),
        .B6(UIO_TOP_UOUT[93]),
        .B5(UIO_TOP_UOUT[94]),
        .B4(UIO_TOP_UOUT[95]),
        .B3(UIO_TOP_UOUT[96]),
        .B2(UIO_TOP_UOUT[97]),
        .B1(UIO_TOP_UOUT[98]),
        .A_I(UIO_TOP_UOUT[99]),
        .A6(UIO_TOP_UOUT[100]),
        .A5(UIO_TOP_UOUT[101]),
        .A4(UIO_TOP_UOUT[102]),
        .A3(UIO_TOP_UOUT[103]),
        .A2(UIO_TOP_UOUT[104]),
        .A1(UIO_TOP_UOUT[105]),
        .H_O(uprj_hpc_out[128]),
        .COUT(uprj_hpc_out[129]),
        .HQ2(uprj_hpc_out[130]),
        .HQ(uprj_hpc_out[131]),
        .HMUX(uprj_hpc_out[132]),
        .G_O(uprj_hpc_out[133]),
        .GQ2(uprj_hpc_out[134]),
        .GQ(uprj_hpc_out[135]),
        .GMUX(uprj_hpc_out[136]),
        .F_O(uprj_hpc_out[137]),
        .FQ2(uprj_hpc_out[138]),
        .FQ(uprj_hpc_out[139]),
        .FMUX(uprj_hpc_out[140]),
        .E_O(uprj_hpc_out[141]),
        .EQ2(uprj_hpc_out[142]),
        .EQ(uprj_hpc_out[143]),
        .EMUX(uprj_hpc_out[144]),
        .D_O(uprj_hpc_out[145]),
        .DQ2(uprj_hpc_out[146]),
        .DQ(uprj_hpc_out[147]),
        .DMUX(uprj_hpc_out[148]),
        .C_O(uprj_hpc_out[149]),
        .CQ2(uprj_hpc_out[150]),
        .CQ(uprj_hpc_out[151]),
        .CMUX(uprj_hpc_out[152]),
        .B_O(uprj_hpc_out[153]),
        .BQ2(uprj_hpc_out[154]),
        .BQ(uprj_hpc_out[155]),
        .BMUX(uprj_hpc_out[156]),
        .A_O(uprj_hpc_out[157]),
        .AQ2(uprj_hpc_out[158]),
        .AQ(uprj_hpc_out[159]),

    //Tile IO ports from BELs
        //input UserCLK,
        //output UserCLKo,
        .FrameData(UIO_TOP_UOUT[137:106]), //CONFIG_PORT
        //output [FrameBitsPerRow -1:0] FrameData_O,
        .FrameStrobe(UIO_TOP_UOUT[158:138]) //CONFIG_PORT
    );
    assign uprj_hpc_out[255:160] = 1'b0;

    // botto modules
    uart_clock uart_clock_i (
        .i_clk(user_clk),
        .i_reset(user_reset),
        .i_sampling_delay(UIO_BOT_UOUT[63:32]),
        .o_clk(uprj_I[0])
    );
    assign uprj_T[0] = 1'b0;

    wire usb_oe;
    usb_cdc usb_i (
      .app_clk_i(user_clk),
      .clk_i(user_clk),
      .rstn_i(~user_reset),

      .configured_o(uprj_hpc_out[256]),

      .dn_rx_i(uprj_O[1]),
      .dn_tx_o(uprj_I[1]),
      .dp_pu_o(uprj_I[3]),
      .dp_rx_i(uprj_O[2]),
      .dp_tx_o(uprj_I[2]),
      .tx_en_o(usb_oe),

      .frame_o(uprj_hpc_out[267:257]),
      .in_data_i(UIO_TOP_UOUT[39:32]),
      .in_ready_o(uprj_hpc_out[268]),
      .in_valid_i(UIO_TOP_UOUT[40]),
      .out_data_o(uprj_hpc_out[276:269]),
      .out_ready_i(UIO_TOP_UOUT[41]),
      .out_valid_o(uprj_hpc_out[277])
    );
    assign uprj_hpc_out[383:278] = 1'b0;


    assign uprj_T[3:1] = {1'b0, ~usb_oe, ~usb_oe};

    assign uprj_hpc_out[511:384] = 1'b0;
    assign UIO_BOT_UIN[159:0] = 1'b0;
    assign uprj_I[17:4] = {13{1'b1}};
    assign uprj_T[17:4] = {13{1'b1}};


    assign sys_gpio_oeb[6:0] = 7'b00011111;
    assign sys_gpio_oeb[24:7] = uprj_T;
    assign sys_gpio_oeb[36:25] = fab_T_top;
    assign sys_gpio_oeb[43:37] = 7'b1111111;

    assign sys_gpio_ieb[6:0] = 7'b1110001;
    assign sys_gpio_ieb[24:7] = ~uprj_T;
    assign sys_gpio_ieb[36:25] = ~fab_T_top;
    assign sys_gpio_ieb[43:37] = 7'b0000001;

    assign sys_gpio_out[6:0] = {done_led, heart_led, cfg_rxled, 4'b0};
    assign sys_gpio_out[24:7] = uprj_I;
    assign sys_gpio_out[36:25] = fab_I_top;
    assign sys_gpio_out[43:37] = 7'b0;

    assign uprj_O = sys_gpio_in[24:7];
    assign fab_O_top = sys_gpio_in[36:25];

endmodule
