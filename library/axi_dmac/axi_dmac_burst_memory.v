// ***************************************************************************
// ***************************************************************************
// Copyright 2014 - 2017 (c) Analog Devices, Inc. All rights reserved.
//
// In this HDL repository, there are many different and unique modules, consisting
// of various HDL (Verilog or VHDL) components. The individual modules are
// developed independently, and may be accompanied by separate and unique license
// terms.
//
// The user should read each of these license terms, and understand the
// freedoms and responsibilities that he or she has by using this source/core.
//
// This core is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
// A PARTICULAR PURPOSE.
//
// Redistribution and use of source or resulting binaries, with or without modification
// of this file, are permitted under one of the following two license terms:
//
//   1. The GNU General Public License version 2 as published by the
//      Free Software Foundation, which can be found in the top level directory
//      of this repository (LICENSE_GPL2), and also online at:
//      <https://www.gnu.org/licenses/old-licenses/gpl-2.0.html>
//
// OR
//
//   2. An ADI specific BSD license, which can be found in the top level directory
//      of this repository (LICENSE_ADIBSD), and also on-line at:
//      https://github.com/analogdevicesinc/hdl/blob/master/LICENSE_ADIBSD
//      This will allow to generate bit files and not release the source code,
//      as long as it attaches to an ADI device.
//
// ***************************************************************************
// ***************************************************************************

module axi_dmac_burst_memory #(
  parameter DATA_WIDTH_SRC = 64,
  parameter DATA_WIDTH_DEST = 64,
  parameter ID_WIDTH = 3,
  parameter MAX_BYTES_PER_BURST = 128,
  parameter ASYNC_CLK = 1
) (
  input src_clk,
  input src_reset,

  input src_data_valid,
  output src_data_ready,
  input [DATA_WIDTH_SRC-1:0] src_data,
  input src_data_last,

  input dest_clk,
  input dest_reset,

  output dest_data_valid,
  input dest_data_ready,
  output [DATA_WIDTH_DEST-1:0] dest_data,
  output dest_data_last,

  output [ID_WIDTH-1:0] dest_request_id,
  input [ID_WIDTH-1:0] dest_data_request_id,
  output [ID_WIDTH-1:0] dest_data_response_id
);

localparam DATA_WIDTH = DATA_WIDTH_SRC > DATA_WIDTH_DEST ?
  DATA_WIDTH_SRC : DATA_WIDTH_DEST;

/* A burst can have up to 256 beats */
localparam BURST_LEN = MAX_BYTES_PER_BURST / (DATA_WIDTH / 8);
localparam BURST_LEN_WIDTH = BURST_LEN > 128 ? 8 :
  BURST_LEN > 64 ? 7 :
  BURST_LEN > 32 ? 6 :
  BURST_LEN > 16 ? 5 :
  BURST_LEN > 8 ? 4 :
  BURST_LEN > 4 ? 3 :
  BURST_LEN > 2 ? 2 : 1;

localparam ADDRESS_WIDTH = BURST_LEN_WIDTH + ID_WIDTH - 1;

localparam AUX_FIFO_SIZE = 2**(ID_WIDTH-1);

`include "inc_id.h"

wire [ID_WIDTH-1:0] dest_src_id;
wire [ID_WIDTH-1:0] src_dest_id;

wire src_beat;
wire src_last_beat;
wire dest_beat;
wire dest_last_beat;
wire dest_last;

wire [ADDRESS_WIDTH-1:0] dest_raddr;
wire [ADDRESS_WIDTH-1:0] src_waddr;

reg [BURST_LEN_WIDTH-1:0] dest_burst_len;

reg [ID_WIDTH-1:0] src_id = 'h0;
reg [ID_WIDTH-1:0] src_id_next;
reg [BURST_LEN_WIDTH-1:0] src_beat_counter = 'h00;
reg src_id_reduced_msb = 1'b0;

wire [ID_WIDTH-2:0] src_id_reduced;

reg [ID_WIDTH-1:0] dest_id_next = 'h0;
reg dest_id_reduced_msb_next = 1'b0;
reg dest_id_reduced_msb = 1'b0;
reg [ID_WIDTH-1:0] dest_id = 'h0;
reg [BURST_LEN_WIDTH-1:0] dest_beat_counter = 'h00;

wire [ID_WIDTH-2:0] dest_id_reduced_next;
wire [ID_WIDTH-1:0] dest_id_next_inc;
wire [ID_WIDTH-2:0] dest_id_reduced;

wire dest_burst_ready;

reg dest_valid = 1'b0;

reg [DATA_WIDTH-1:0] ram[0:2**ADDRESS_WIDTH-1];

reg [BURST_LEN_WIDTH-1:0] burst_len_mem[0:2**(ID_WIDTH-1)-1];

assign src_id_reduced = {src_id_reduced_msb,src_id[ID_WIDTH-3:0]};

wire src_mem_data_valid;
reg src_mem_data_ready;
wire src_mem_data_last;
wire [DATA_WIDTH-1:0] src_mem_data;

wire dest_mem_data_ready;
reg dest_mem_data_valid = 1'b0;
reg dest_mem_data_last = 1'b0;
reg [DATA_WIDTH-1:0] dest_mem_data = 'h0;

assign src_beat = src_mem_data_valid & src_mem_data_ready;
assign src_last_beat = src_beat & src_mem_data_last;
assign src_waddr = {src_id_reduced,src_beat_counter};

axi_dmac_resize_src #(
  .DATA_WIDTH_SRC (DATA_WIDTH_SRC),
  .DATA_WIDTH_MEM (DATA_WIDTH)
) i_resize_src (
  .clk (src_clk),
  .reset (src_reset),
  .src_data_valid(src_data_valid),
  .src_data_ready(src_data_ready),
  .src_data (src_data),
  .src_data_last (src_data_last),

  .mem_data_valid(src_mem_data_valid),
  .mem_data_ready(src_mem_data_ready),
  .mem_data (src_mem_data),
  .mem_data_last (src_mem_data_last)
);

always @(*) begin
  if (src_last_beat == 1'b1) begin
    src_id_next <= inc_id(src_id);
  end else begin
    src_id_next <= src_id;
  end
end

always @(posedge src_clk) begin
  /* Ready if there is room for at least one full burst. */
  src_mem_data_ready <= (src_id_next[ID_WIDTH-1] == src_dest_id[ID_WIDTH-1] ||
                src_id_next[ID_WIDTH-2] == src_dest_id[ID_WIDTH-2] ||
                src_id_next[ID_WIDTH-3:0] != src_dest_id[ID_WIDTH-3:0]);
end

always @(posedge src_clk) begin
  if (src_reset == 1'b1) begin
    src_id <= 'h00;
    src_id_reduced_msb <= 1'b0;
  end else begin
    src_id <= src_id_next;
    src_id_reduced_msb <= ^src_id_next[ID_WIDTH-1-:2];
  end
end

always @(posedge src_clk) begin
  if (src_reset == 1'b1) begin
    src_beat_counter <= 'h00;
  end else if (src_beat == 1'b1) begin
    src_beat_counter <= src_beat_counter + 1'b1;
  end
end

always @(posedge src_clk) begin
  if (src_beat == 1'b1) begin
    ram[src_waddr] <= src_mem_data;
  end
end

always @(posedge src_clk) begin
  if (src_last_beat == 1'b1) begin
    burst_len_mem[src_id_reduced] <= src_beat_counter;
  end
end

assign dest_id_reduced_next = {dest_id_reduced_msb_next,dest_id_next[ID_WIDTH-3:0]};
assign dest_id_reduced = {dest_id_reduced_msb,dest_id[ID_WIDTH-3:0]};

assign dest_ready = ~dest_mem_data_valid | dest_mem_data_ready;
assign dest_last = dest_beat_counter == dest_burst_len;

assign dest_beat = dest_valid & dest_ready;
assign dest_last_beat = dest_last & dest_beat;
assign dest_raddr = {dest_id_reduced,dest_beat_counter};

assign dest_burst_ready = ~dest_valid | dest_last_beat;

always @(posedge dest_clk) begin
  if (dest_reset == 1'b1) begin
    dest_valid <= 1'b0;
  end else if (dest_data_request_id != dest_id_next) begin
    dest_valid <= 1'b1;
  end else if (dest_last_beat == 1'b1) begin
    dest_valid <= 1'b0;
  end
end

always @(posedge dest_clk) begin
  if (dest_reset == 1'b1) begin
    dest_mem_data_valid <= 1'b0;
  end else if (dest_valid == 1'b1) begin
    dest_mem_data_valid <= 1'b1;
  end else if (dest_mem_data_ready == 1'b1) begin
    dest_mem_data_valid <= 1'b0;
  end
end

/*
 * This clears dest_data_last after the last beat. Strictly speaking this is not
 * necessary if this followed AXI handshaking rules since dest_data_last would
 * be qualified by dest_data_valid and it is OK to retain the previous value of
 * dest_data_last when dest_data_valid is not asserted. But clearing the signal
 * here doesn't cost much and can simplify some of the more congested
 * combinatorical logic further up the pipeline since we can assume that
 * fifo_last == 1'b1 implies fifo_valid == 1'b1.
 */
always @(posedge dest_clk) begin
  if (dest_reset == 1'b1) begin
    dest_mem_data_last <= 1'b0;
  end else if (dest_beat == 1'b1) begin
    dest_mem_data_last <= dest_last;
  end else if (dest_mem_data_ready == 1'b1) begin
    dest_mem_data_last <= 1'b0;
  end
end

assign dest_id_next_inc = inc_id(dest_id_next);

always @(posedge dest_clk) begin
  if (dest_reset == 1'b1) begin
    dest_id_next <= 'h00;
    dest_id_reduced_msb_next <= 1'b0;
  end else if (dest_data_request_id != dest_id_next &&
               dest_burst_ready == 1'b1) begin
    dest_id_next <= dest_id_next_inc;
    dest_id_reduced_msb_next <= ^dest_id_next_inc[ID_WIDTH-1-:2];
  end
end

always @(posedge dest_clk) begin
  if (dest_burst_ready == 1'b1) begin
    dest_burst_len <= burst_len_mem[dest_id_reduced_next];
  end
end

always @(posedge dest_clk) begin
  if (dest_burst_ready == 1'b1) begin
    dest_id <= dest_id_next;
    dest_id_reduced_msb <= dest_id_reduced_msb_next;
  end
end

always @(posedge dest_clk) begin
  if (dest_reset == 1'b1) begin
    dest_beat_counter <= 'h00;
  end else if (dest_beat == 1'b1) begin
    dest_beat_counter <= dest_beat_counter + 1'b1;
  end
end

always @(posedge dest_clk) begin
  if (dest_beat == 1'b1) begin
    dest_mem_data <= ram[dest_raddr];
  end
end

axi_dmac_resize_dest #(
  .DATA_WIDTH_DEST (DATA_WIDTH_DEST),
  .DATA_WIDTH_MEM (DATA_WIDTH)
) i_resize_dest (
  .clk (dest_clk),
  .reset (dest_reset),

  .mem_data_valid (dest_mem_data_valid),
  .mem_data_ready (dest_mem_data_ready),
  .mem_data (dest_mem_data),
  .mem_data_last (dest_mem_data_last),

  .dest_data_valid(dest_data_valid),
  .dest_data_ready(dest_data_ready),
  .dest_data (dest_data),
  .dest_data_last (dest_data_last)
);

sync_bits #(
  .NUM_OF_BITS(ID_WIDTH),
  .ASYNC_CLK(ASYNC_CLK)
) i_dest_sync_id (
  .in(src_id),
  .out_clk(dest_clk),
  .out_resetn(1'b1),
  .out(dest_src_id)
);

sync_bits #(
  .NUM_OF_BITS(ID_WIDTH),
  .ASYNC_CLK(ASYNC_CLK)
) i_src_sync_id (
  .in(dest_id),
  .out_clk(src_clk),
  .out_resetn(1'b1),
  .out(src_dest_id)
);

assign dest_request_id = dest_src_id;
assign dest_data_response_id = dest_id;

endmodule
