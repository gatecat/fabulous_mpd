// SPDX-FileCopyrightText: 2020 Efabless Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// SPDX-License-Identifier: Apache-2.0

`default_nettype none
/*
 *---------------------------------------------------------------------
 *
 * Based on gpio_wb.v; but for fabric-configured IO
 *
 * Control bits (11):
 *
 *  11				output value
 *  10				output enable value
 *   9				input enable value
 *   8				override output
 *   7  			override output enable
 *   6				override input enable
 *   5  pad_gpio_slow_sel	Slow output slew select
 *   4	pad_gpio_vtrip_sel	Trip point voltage select
 *   3  pad_gpio_ib_mode_sel	I-B mode select
 * 2-0	pad_gpio_dm		Digital mode
 *
 * The MSB of GPIO_DEFAULTS is used to select whether IO configuration from fabric is allowed
 *
 *---------------------------------------------------------------------
 */

module mpd_io_ctrl #(
    parameter GPIO_DEFAULTS = 13'h001
) (
    input fabric_done,
    input [11:0] fabric_config,

    output pad_gpio_slow_sel,
    output pad_gpio_vtrip_sel,
    output pad_gpio_ib_mode_sel,
    output [2:0] pad_gpio_dm,

    input  pad_gpio_in,
    output pad_gpio_out,
    output pad_gpio_oeb,
    output pad_gpio_ieb,

    output sys_gpio_in,
    input  sys_gpio_out,
    input  sys_gpio_oeb,
    input  sys_gpio_ieb
);

    wire [11:0] active_cfg = (fabric_done & GPIO_DEFAULTS[12]) ? fabric_config : GPIO_DEFAULTS[11:0];

    /* Internally registered signals */
    wire gpio_out_override;
    wire gpio_out_value;
    wire gpio_oeb_override;
    wire gpio_oeb_value;
    wire gpio_ieb_override;
    wire gpio_ieb_value;


    assign gpio_out_value	 = active_cfg[11];
    assign gpio_oeb_value	 = active_cfg[10];
    assign gpio_ieb_value	 = active_cfg[9];
    assign gpio_out_override	 = active_cfg[8];
    assign gpio_oeb_override	 = active_cfg[7];
    assign gpio_ieb_override	 = active_cfg[6];
    assign pad_gpio_slow_sel    = active_cfg[5];
    assign pad_gpio_vtrip_sel   = active_cfg[4];
    assign pad_gpio_ib_mode_sel = active_cfg[3];
    assign pad_gpio_dm          = active_cfg[2:0];

    assign sys_gpio_in = pad_gpio_in;
    assign pad_gpio_out = (gpio_out_override) ? gpio_out_value : sys_gpio_out;
    assign pad_gpio_oeb = (gpio_oeb_override) ? gpio_oeb_value : sys_gpio_oeb;
    assign pad_gpio_ieb = (gpio_ieb_override) ? gpio_ieb_value : sys_gpio_ieb;
		
endmodule
`default_nettype wire
