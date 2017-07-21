// UART-UART test
// Send multiple packets using polling
disable_rx = 1'b1;

$display("Set RX Baud to 57600");
set_dlab(UART1, 1'b1);
//cpu_wr_reg(DLL, 8'hF7);
cpu_wr_reg(UART1, DLL, 8'hF5);
cpu_wr_reg(UART1, DLM, 8'h00);

$display("unSet Auto Flow control, force RTS");
cpu_wr_reg(UART1, MCR1, 8'h02);

$display("Set FIFO control for a threshold of 4");
cpu_wr_reg(UART1, IIR_FCR1, 8'h81);

//$display("Set RX interrupt enable on fifo threshold");
//cpu_wr_reg(UART1, IIR_FCR1, 8'h81);

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

   $display("Test LSR register");
   // we only care about bit 0, but bits 6&5 should be set as TX is idle
   cpu_rd_reg_verify(UART0, LSR0, 8'h60, 8'hFF); // Check for exact match

   for (j = 0; j < 9; j = j + 1) begin
      // Excercise the FIFO, write in 4 words every time it's empty
      send_data[UART0] = $random;
      data_shift[addr_in] = send_data[UART0];
      addr_in = addr_in + 1;
      $display("Write %h over the UART", send_data[UART0]);
      set_dlab(UART0, 1'b0);
      cpu_wr_data(UART0, send_data[UART0]);
   end

   // Test RX for overrun
   cpu_rd_reg(UART1, LSR0);
   while (~test_reg[UART1][1]) begin
      cpu_rd_reg(UART1, LSR0);
      //if (uart_rtsn_1) $display("CTS set, overrun won't occur");
      //break;
   end
   if (test_reg[UART1][1]) $display("PASS: RX FIFO Overrun.");





