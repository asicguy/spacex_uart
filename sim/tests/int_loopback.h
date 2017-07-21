// Simple loopback test
// Send single random message with the different parity modes

$display("Set Baud to 57600");
set_dlab(UART0, 1'b1);
//cpu_wr_reg(DLL, 8'hF7);
cpu_wr_reg(UART0, DLL, 8'hF5);
cpu_wr_reg(UART0, DLM, 8'h00);

$display("Test No parity mode");
$display("Set Internal Loopback and Auto Flow control");
cpu_wr_reg(UART0, MCR1, 8'h30);

$display("Test LSR register");
// we only care about bit 0, but bits 6&5 should be set as TX is idle
cpu_rd_reg_verify(UART0, LSR0, 8'h60, 8'hFF); // Check for exact match

send_data[UART0] = $random;
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

// parity == 001
$display("Test Odd parity mode");
cpu_wr_reg(UART0, LCR0, {2'b0, 3'b001, 3'b0});

$display("Test LSR register");
// we only care about bit 0, but bits 6&5 should be set as TX is idle
cpu_rd_reg_verify(UART0, LSR0, 8'h60, 8'hFF); // Check for exact match

send_data[UART0] = $random;
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

// parity == 011
$display("Test Even parity mode");
cpu_wr_reg(UART0, LCR0, {2'b0, 3'b011, 3'b0});

$display("Test LSR register");
// we only care about bit 0, but bits 6&5 should be set as TX is idle
cpu_rd_reg_verify(UART0, LSR0, 8'h60, 8'hFF); // Check for exact match

send_data[UART0] = $random;
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




