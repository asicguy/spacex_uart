///////////////////////////////////////////////////////////////////////////////
//
//  Copyright (C) 2014 Francis Bruno, All Rights Reserved
// 
//  This program is free software; you can redistribute it and/or modify it 
//  under the terms of the GNU General Public License as published by the Free 
//  Software Foundation; either version 3 of the License, or (at your option) 
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but 
//  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
//  or FITNESS FOR A PARTICULAR PURPOSE. 
//  See the GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License along with
//  this program; if not, see <http://www.gnu.org/licenses>.
//
//  This code is available under licenses for commercial use. Please contact
//  Francis Bruno for more information.
//
//  http://www.gplgpu.com
//  http://www.asicsolutions.com
//  
//  Title       :  Simple UART CPU interface
//  File        :  uart_cpu.v
//  Author      :  Frank Bruno
//  Created     :  28-May-2015
//  RCS File    :  $Source:$
//  Status      :  $Id:$
//
//
///////////////////////////////////////////////////////////////////////////////
//
//  Description :
//  CPU interface for simple UART core
//
//////////////////////////////////////////////////////////////////////////////
//
//  Modules Instantiated:
//
///////////////////////////////////////////////////////////////////////////////
//
//  Modification History:
//
//  $Log:$
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 10ps

module uart_cpu
  (
   // Utility signals
   input 	     sys_clk, // 100 Mhz for this example
   input 	     sys_rstn, // Active low reset
   input 	     baud_en, // Enable for the baud clock
   
   // CPU interface
   input 	     cpu_csn, // Chip select for CPU interface
   input 	     cpu_rdwrn, // 1 = Read, 0 = write
   input [2:0] 	     cpu_addr, // Address bus
   input [7:0] 	     cpu_din, // Data into core from CPU
   output 	     cpu_ready, // CPU accepts request
   output reg [7:0]  cpu_dout, // Data from Core to CPU
   output reg 	     cpu_valid, // Read is valid
   output reg	     cpu_int, // interrupt

   // Registers to design
   output reg [15:0] baud_terminal_count, // Terminal count for baud en
   output reg [2:0]  parity, // Parity setting
   output reg 	     force_rts, // Force RTS value for testing
   output reg 	     autoflow, // Generate RTS/ CTS automatically
   output reg 	     loopback, // Loopback for test
   output reg 	     baud_reset, // Reset baud rate counter
   
   // RX interface
   input 	     rx_fifo_push, // Push data from RX interface
   input [12:0]      rx_fifo_din, // Data from RX interface
   input             rx_break_det, // Detect break (not implemented)
   input 	     rx_parity_err, // Parity error flag on RX
   input 	     rx_frame_err, // Stop bit not valid
   output 	     rx_fifo_full, // FIFO Full
   
   // TX interface
   input 	     tx_fifo_pop, // Pop TX data for sending
   input 	     tx_shift_empty, // TX shift register is empty
   output 	     tx_data_avail, // ~tx_fifo_empty
   output [12:0]     tx_fifo_dout, // Fifo data for TX
   
   // External pins
   input 	     uart_cts // Clear to send
   );
   
   localparam
     // mapped to 16550a registers
     RBR_THR    = 4'h0, // RX Register, TX Register - DLL LSB
     IER_IER    = 4'h1, // Interrupt Enable Register
     IIR_FCR0   = 4'h2, // interrupt ID register, FIFO control register
     LCR0       = 4'h3, // Line Control Register
     MCR0       = 4'h4, // Line Control Register
     LSR0       = 4'h5, // Line Status Register
     MSR0       = 4'h6, // Modem Status Register
     SCR0       = 4'h7, // Scratch register
     DLL        = 4'h8, // Divisor LSB
     DLM        = 4'h9, // Divisor MSB
     // These registers set the baud rate:
     // Baud    BAUDCNT_HI BAUDCNT_LO   %ERR
     // 300     8'hB9      8'hFF      -0.006382%
     // 1200    8'h2E      8'h7F      -0.006325%
     // 2400    8'h17      8'h3F      -0.00625%
     // 4800    8'h0B      8'h9F      -0.0061%      
     // 9600    8'h05      8'hCF      -0.0058%
     // 14400   8'h03      8'hDF      -0.0055%
     // 19200   8'h02      8'hE7      -0.0052%
     // 28800   8'h01      8'hEF      -0.0046%
     // 38400   8'h01      8'h73      -0.004%
     // 57600   8'h00      8'hF7      -0.0028%
     IIR_FCR1   = 4'hA, // interrupt ID register, FIFO control register
     LCR1       = 4'hB, // Line Control Register
     MCR1       = 4'hC, // Modem Control Register
     LSR1       = 4'hD, // Line Status Register
     MSR1       = 4'hE, // Modem Status Register
     SCR1       = 4'hF; // Scratch register

   localparam
     CPU_IDLE    = 2'h0,
     CPU_RD_WAIT = 2'h1,
     CPU_RD_DONE = 2'h2;
   
   reg 		     dlab;               // Register selector
   reg 		     break_en;           // Enable break
   reg 		     fifo_enable;        // Enable the FIFOs
   reg 		     reset_rx_fifo;      // Reset RX FIFO
   reg 		     reset_tx_fifo;      // Reset TX FIFO
   reg [7:0] 	     scratch_reg;        // For backward compatibility
   reg               cts_change;         // CTS has changed
   reg               cts_last;           // last CTS value
   
   wire [12:0] 	     rx_fifo_out;        // Data from the FIFO
   reg 		     rx_fifo_pop;        // Pop data from the RX fifo
   wire 		     rx_fifo_empty;      // FIFO empty
   wire 	     rx_data_avail;      // ~fifo empty
   reg [3:0] 	     rx_fifo_count;      // {full, count[2:0]}
   reg [2:0] 	     rx_thresh;          // Low watermark
   reg 		     rx_toggle;          // toggle every RX data
   reg 		     tx_toggle;          // toggle every RX data

   // Interrupt enables
   reg 		     en_rx_data_avail;   // ~fifo empty
   reg               en_tx_fifo_empty;   // 
   reg 		     en_rx_status_change;
   reg 		     en_msr_change;

   wire              tx_fifo_empty;
   reg 		     rx_status_change;
   reg 		     msr_change;
   reg 		     thr_empty, tx_shift_empty_d;
   reg [2:0] 	     int_status;    // For interrupt readback
   //Bits 1 and 2	Bit 2	Bit 1	
   //0	0	Modem Status Interrupt (lowest)
   //0	1	Transmitter Holding Register Empty Interrupt
   //1	0	Received Data Available Interrupt
   //1	1	Receiver Line Status Interrupt (higest)
   wire 	     char_timeout = 0; // fixme!!!
   wire 	     frame_err;     // Capture the framing error
   
   reg               tx_fifo_push;  // Push data into TX FIFO
   reg [12:0]        tx_fifo_din;   // Registered data into FIFO
   //    

   wire		     break_det;     // Break Interrupt
   reg 		     overrun_error; // write to full RX fifo
   wire              parity_err;
   wire              rx_fifo_error;
   reg [1:0]         cpu_state;     // CPU state variable
   
   // Qualify read fixme!!!!
   assign cpu_ready 	= ~cpu_csn & (cpu_rdwrn | ~cpu_rdwrn);
   always @(posedge sys_clk) begin

      // defaults
      rx_fifo_pop   <= 1'b0;
      tx_fifo_push  <= 1'b0;
      reset_rx_fifo <= 1'b0;
      reset_tx_fifo <= 1'b0;
      cpu_valid     <= 1'b0;
      baud_reset    <= 1'b0;
      
      // Detect a change in CTS status
      cts_last <= uart_cts;
      if (baud_en & (cts_last ^ uart_cts)) cts_change <= 1'b1;
      
      // Detect overrun
      if (rx_fifo_push & ~rx_fifo_pop & rx_fifo_full) overrun_error <= 1'b1;
      else if (~rx_fifo_full) overrun_error <= 1'b0;

      // set int_status and cpu_int
      if (en_rx_status_change && (overrun_error || 
				  parity_err & ~rx_fifo_empty ||
				  break_det & ~rx_fifo_empty ||
				  frame_err & ~rx_fifo_empty))
	{int_status, cpu_int} <= 4'b0110;
      else if (en_rx_data_avail && rx_data_avail)
	// This might be a cheat, but I didn't see the purpose of setting
	// a threshold and going off even if 1 piece of data was in the FIFO
	// I might have read the spec wrong. This is better anyways.
	{int_status, cpu_int} <= 4'b0100;
      else if (char_timeout)
	// fixme!!!!
	{int_status, cpu_int} <= 4'b1100;
      else if (en_tx_fifo_empty && thr_empty)
	// fixme, set a flag when go empty and clear when
	// reading this
	{int_status, cpu_int} <= 4'b0010;
      else if (en_msr_change && cts_change)
	{int_status, cpu_int} <= 4'b0000;
      else
	cpu_int <= 1'b1;
      
      // detect shift register going empty and set thr_empty
      tx_shift_empty_d <= tx_shift_empty;
      if (tx_shift_empty & ~tx_shift_empty_d && tx_fifo_empty) thr_empty <= 1'b1;
      
      case (cpu_state)
	CPU_IDLE: begin
	   if (~cpu_csn)
	     if (cpu_rdwrn) begin
		cpu_valid     <= 1'b1;

		// Read bus
		case ({dlab, cpu_addr})
		  // RX Buffer Register, TX Holding Register
		  RBR_THR: begin
		     cpu_dout 	  <= rx_toggle ? rx_fifo_out[12:8] : 
				     rx_fifo_out[7:0];
		     rx_toggle 	  <= ~rx_toggle;
		     if (rx_toggle && ~rx_fifo_empty) begin
			cpu_valid   <= 1'b0;
			rx_fifo_pop <= 1'b1;
			cpu_state   <= CPU_RD_WAIT;
		     end
		  end
		  IER_IER: cpu_dout <= {4'h0, // Don't support lp modes or sleep
					en_rx_data_avail,
					en_tx_fifo_empty,
					en_rx_status_change,
					en_msr_change};
		  IIR_FCR0, IIR_FCR1: begin
		     thr_empty <= 1'b0; // reset status bit
		     cpu_dout[7:6] <= {2{fifo_enable}};
		     cpu_dout[5:4] <= 2'b00;
		     cpu_dout[3:0] <= {int_status, cpu_int};
		  end
		  LCR0, LCR1: begin
		     cpu_dout <= {dlab,     // 1 = select config registers, 0 = normal
				  break_en, // Enable break signal (not currently used)
				  parity,   /* Parity setting
					     * [5:3]    Setting
					     *  xx0     No Parity
					     *  001     Odd Parity
					     *  011     Even Parity
					     *  101     High Parity (stick)
					     *  111     Low Parity (stick)
					     */
				  1'b0,     // Unused (not requested)
				  2'h0};    // Unused since we are forcing 13 bit data
		  end // case: LCR0, LCR1
		  MCR0, MCR1: begin
		     cpu_dout <= {2'b0,       // Reserved
				  autoflow,   // Generate RTS automatically
				  loopback,   // Loopback mode
				  2'b0,       // AUX unused
				  force_rts,  // RTS
				  1'b0};      // DTR unused
		  end
		  LSR0, LSR1: begin
		     cpu_dout <= {rx_fifo_error & ~rx_fifo_empty, // Error in Received FIFO (br, par, fr)
				  tx_shift_empty,  // Empty Data Holding Registers
				  tx_fifo_empty, // Empty Transmitter Holding Register
				  break_det & ~rx_fifo_empty, // Break Interrupt
				  frame_err & ~rx_fifo_empty, // Framing Error
				  parity_err & ~rx_fifo_empty, // Parity Error
				  overrun_error, // Overrun Error
				  ~rx_fifo_empty};   // Data Ready
		  end
		  MSR0, MSR1: begin
		     cts_change <= 1'b0;
		     cpu_dout <= {3'h0,        // Unused
				  uart_cts,    // current Clear to send
				  3'h0,
				  cts_change}; // Change in CTS detected
		  end
		  SCR0, SCR1: cpu_dout <= scratch_reg; // Readback scratch
		  DLL: begin
		     cpu_dout  <= baud_terminal_count[7:0];
		     baud_reset <= 1'b1;
		  end
		  DLM: begin
		     cpu_dout  <= baud_terminal_count[15:8];
		     baud_reset <= 1'b1;
		  end
		  default:    cpu_dout <= 8'h0; // Not necessary
		endcase // case (cpu_addr)
		
	     end else begin
		case ({dlab, cpu_addr})
		  RBR_THR: begin
		     thr_empty <= 1'b0;
		     // RX Buffer Register, TX Holding Register
		     tx_toggle <= ~tx_toggle;
		     if (tx_toggle) begin
			tx_fifo_din[12:8] <= cpu_din[4:0];
			tx_fifo_push      <= 1'b1;
		     end else
		       tx_fifo_din[7:0]   <= cpu_din;
		  end
		  IER_IER: begin
		     {en_rx_data_avail,
		      en_tx_fifo_empty,
		      en_rx_status_change,
		      en_msr_change} <= cpu_din[3:0];
		  end
		  IIR_FCR0, IIR_FCR1: begin
		     // FIFO control register
		     fifo_enable   <= cpu_din[0];
		     if (cpu_din[1]) begin
			reset_rx_fifo <= 1'b1;
			rx_toggle     <= 1'b0;
		     end
		     if (cpu_din[2]) begin
			reset_tx_fifo <= 1'b1;
			tx_toggle     <= 1'b0;
		     end
		     // cpu_din[3] DMA mode, not supported currently
		     // cpu_din[4] Reserved
		     // cpu_din[5] Reserved
		     // Threshold set for RX. 1/2 the 16 FIFO of 16550
		     case (cpu_din[7:6])
		       2'h0: rx_thresh <= 4'h1;
		       2'h1: rx_thresh <= 4'h2;
		       2'h2: rx_thresh <= 4'h4;
		       2'h3: rx_thresh <= 4'h7;
		     endcase // case (cpu_din[7:6])
		  end // case: IIR_FCR0,...
		  LCR0, LCR1: begin
		     dlab     <= cpu_din[7]; // 1 = select config registers
		     break_en <= cpu_din[6]; // (not currently used)
		     parity   <= cpu_din[5:3];   /* Parity setting
						  * [5:3]    Setting
						  *  xx0     No Parity
						  *  001     Odd Parity
						  *  010     Even Parity
						  *  101     High Parity (stick)
						  *  111     Low Parity (stick)
						  */
		  end // case: LCR0, LCR1
		  MCR0, MCR1: begin
		     autoflow  <= cpu_din[5]; // Generate RTS automatically
		     loopback  <= cpu_din[4]; // Loopback mode
		     force_rts <= cpu_din[1]; // RTS
		  end
		  SCR0, SCR1: scratch_reg <= cpu_din; // scratch register
		  DLL:        baud_terminal_count[7:0]  <= cpu_din;
		  DLM:        baud_terminal_count[15:8] <= cpu_din;
		endcase // case (cpu_addr)
	     end // else: !if(~rx_fifo_empty)
	end // case: CPU_IDLE
	CPU_RD_WAIT: cpu_state <= CPU_RD_DONE;
	CPU_RD_DONE: begin
	   cpu_valid   <= 1'b1;
	   cpu_state <= CPU_IDLE;
	end
	
      endcase // case (cpu_state)
      
      // Reset clause
      if (~sys_rstn) begin
	 cpu_state 	     <= CPU_IDLE;
	 thr_empty           <= 1'b0;
	 rx_toggle 	     <= 1'b0;
	 tx_toggle 	     <= 1'b0;
	 dlab                <= 1'b0;
	 break_en            <= 1'b0;
	 parity              <= 3'h0;
	 autoflow            <= 1'b1;
	 loopback            <= 1'b0;
	 force_rts           <= 1'b0;
	 fifo_enable         <= 1'b1;
	 reset_rx_fifo       <= 1'b0;
	 reset_tx_fifo       <= 1'b0;
	 rx_thresh           <= 2'h1;
	 en_rx_data_avail    <= 1'b0;
	 en_tx_fifo_empty    <= 1'b0;
	 en_rx_status_change <= 1'b0;
	 en_msr_change       <= 1'b0;
	 tx_fifo_push        <= 1'b0;
	 rx_fifo_pop         <= 1'b0;
	 overrun_error       <= 1'b0;
	 baud_terminal_count <= 16'd248; // 57600
      end // if (~cpu_rstn)
      
   end // always @ (posedge sys_clk)

   // FIFO blocks
   // These are synchronous due to the nature of the clocks. The presentation
   // will go over asynchronous FIFOs
   // Need to store:
   // framing error
   // parity error
   sync_fifo 
     #
     (
      .DWIDTH        (16)
      )
   u_rx
     (
      // Utility signals
      .sys_clk       (sys_clk),
      .sys_rstn      (sys_rstn),
      .reset_fifo    (reset_rx_fifo),
   
      .fifo_push     (rx_fifo_push),
      .fifo_pop      (rx_fifo_pop),
      .fifo_din      ({rx_break_det,
		       rx_parity_err, 
		       rx_frame_err,
		       rx_fifo_din}),
      .data_thresh   (rx_thresh),
   
      .fifo_out      ({break_det,
		       parity_err,
		       frame_err,
		       rx_fifo_out}),
      .data_avail    (rx_data_avail),
      .fifo_empty    (rx_fifo_empty),
      .fifo_full     (rx_fifo_full)
      );
   
   assign rx_fifo_error = break_det | parity_err | frame_err;
      
   sync_fifo u_tx
     (
      // Utility signals
      .sys_clk       (sys_clk),
      .sys_rstn      (sys_rstn),
      .reset_fifo    (reset_tx_fifo),
   
      .fifo_push     (tx_fifo_push),
      .fifo_pop      (tx_fifo_pop),
      .fifo_din      (tx_fifo_din),
      .data_thresh   (3'h7), // Don't care
   
      .fifo_out      (tx_fifo_dout),
      .data_avail    (),
      .fifo_empty    (tx_fifo_empty),
      .fifo_full     (tx_fifo_full)
      );

   assign tx_data_avail = ~tx_fifo_empty;
   
endmodule // uart
