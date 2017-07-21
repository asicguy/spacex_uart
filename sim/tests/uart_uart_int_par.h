// UART-UART test
// Send multiple packets using polling
par_error_test = 1'b1;
stop_on_fail = 1'b1;

$display("Set RX Baud to 57600");
set_dlab(UART1, 1'b1);
//cpu_wr_reg(DLL, 8'hF7);
cpu_wr_reg(UART1, DLL, 8'hF5);
cpu_wr_reg(UART1, DLM, 8'h00);

$display("Set Auto Flow control");
cpu_wr_reg(UART1, MCR1, 8'h20);

$display("Set FIFO control for a threshold of 4");
cpu_wr_reg(UART1, IIR_FCR1, 8'h81);

$display("Set RX interrupt enable on fifo threshold");
cpu_wr_reg(UART1, IIR_FCR1, 8'h81);

// parity == 001
$display("Test Odd parity mode");
cpu_wr_reg(UART1, LCR0, {2'b0, 3'b001, 3'b0});

// Set interrupt on data available
cpu_wr_reg(UART1, IER_IER, 8'h08);
set_dlab(UART1, 1'b0);

// For polling
//$display("Enabling RX UART");
//enable_uart1_rx = 1'b1;

$display("Set TX Baud to 57600");
set_dlab(UART0, 1'b1);
//cpu_wr_reg(DLL, 8'hF7);
cpu_wr_reg(UART0, DLL, 8'hF5);
cpu_wr_reg(UART0, DLM, 8'h00);

$display("Set Auto Flow control");
cpu_wr_reg(UART0, MCR1, 8'h20);

// parity == 001
$display("Test Odd parity mode");
cpu_wr_reg(UART0, LCR0, {2'b0, 3'b001, 3'b0});

for (i = 0; i < 128; i = i + 1) begin
   $display("Test LSR register");
   // we only care about bit 0, but bits 6&5 should be set as TX is idle
   cpu_rd_reg_verify(UART0, LSR0, 8'h60, 8'hFF); // Check for exact match

   inj_par_err = $random;
   if (inj_par_err) begin
      $display("injecting parity Error");
      cpu_wr_reg(UART0, LCR0, {2'b0, 3'b011, 3'b0});
      par_shift[addr_in] = 1'b1;
   end else begin
      cpu_wr_reg(UART0, LCR0, {2'b0, 3'b001, 3'b0});
      par_shift[addr_in] = 1'b0;
   end

   send_data[UART0] = $random;
   data_shift[addr_in] = send_data[UART0];
   addr_in = addr_in + 1;
   $display("Write %h over the UART", send_data[UART0]);
   set_dlab(UART0, 1'b0);
   cpu_wr_data(UART0, send_data[UART0]);
   
   // Wait for TX to no longer be idle
   cpu_rd_reg(UART0, LSR0);
   while (~test_reg[UART0][5]) cpu_rd_reg(UART0, LSR0);
   $display("TX FIFO has data.");
   
   cpu_rd_reg(UART0, LSR0);
   while (~test_reg[UART0][6]) cpu_rd_reg(UART0, LSR0);
   $display("TX Shifter has data.");

   $display("Test LSR register");
   // we only care about bit 0, but bits 6&5 should be set as TX is idle
   cpu_rd_reg_verify(UART0, LSR0, 8'h60, 8'hFF); // Check for exact match

/*   
   // So now we poll on bit 0
   $display("Poll for RX data Available.");
   cpu_rd_reg(UART0, LSR0);
   while (~test_reg[UART0][0]) cpu_rd_reg(UART0, LSR0);
   
   // Check the stats
   cpu_rd_reg_verify(UART0, LSR0, 8'h61, 8'hFF); // Check for exact match
   
   $display("RX shows data available.");
   if (&test_reg[UART0][6:5]) begin
      $display("TX is now idle again");
   end else begin
      $display("TX is not idle again");
      test_failed = 1'b1;
      if (stop_on_fail) $stop;
   end
   
   // Readback and check the data
   cpu_rd_dat_verify(UART0, send_data[UART0]);
  */
 
end // for (i = 0; i < 128; i = i + 1)




