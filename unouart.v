//    This file is part of the ZXUNO Spectrum core.
//    Creation date is 19:56:26 2015-10-17 by Miguel Angel Rodriguez Jodar
//    (c)2014-2020 ZXUNO association.
//    ZXUNO official repository: http://svn.zxuno.com/svn/zxuno
//    Username: guest   Password: zxuno
//    Github repository for this core: https://github.com/mcleod-ideafix/zxuno_spectrum_core
//
//    ZXUNO Spectrum core is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    ZXUNO Spectrum core is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with the ZXUNO Spectrum core.  If not, see <https://www.gnu.org/licenses/>.
//
//    Any distributed copy of this file must keep this notice intact.


module uart (
    // CPU interface
    input wire clk,
    input wire [7:0] txdata,
    input wire txbegin,
    output wire txbusy,
    output wire [7:0] rxdata,
    output wire rxrecv,
    // RS232 interface
    input wire rx,
    output wire tx
    );

    parameter CLK = 28000000;
    parameter BPS = 115200;

    uart_tx #(.CLK(CLK), .BPS(BPS)) transmitter (
        .clk(clk),
        .txdata(txdata),
        .txbegin(txbegin),
        .txbusy(txbusy),
        .tx(tx)
    );

    uart_rx #(.CLK(CLK), .BPS(BPS)) receiver (
        .clk(clk),
        .rxdata(rxdata),
        .rxrecv(rxrecv),
        .rx(rx)
    );
endmodule

module uart_tx (
    // CPU interface
    input wire clk,
    input wire [7:0] txdata,
    input wire txbegin,
    output wire txbusy,
    // RS232 interface
    output reg tx
    );

    initial tx = 1'b1;

    parameter CLK = 28000000;
    parameter BPS = 115200;
    localparam PERIOD = CLK / BPS;

    localparam
        IDLE  = 2'd0,
        START = 2'd1,
        BIT   = 2'd2,
        STOP  = 2'd3;

    reg [7:0] txdata_reg;
    reg [1:0] state = IDLE;
    reg [31:0] bpscounter;
    reg [2:0] bitcnt;
    reg txbusy_ff = 1'b0;
    assign txbusy = txbusy_ff;

    always @(posedge clk) begin
        if (txbegin == 1'b1 && txbusy_ff == 1'b0 && state == IDLE) begin
            txdata_reg <= txdata;
            txbusy_ff <= 1'b1;
            state <= START;
            bpscounter <= PERIOD;
        end
        if (txbegin == 1'b0 && txbusy_ff == 1'b1) begin
            case (state)
                START:
                    begin
                        tx <= 1'b0;
                        bpscounter <= bpscounter - 16'd1;
                        if (bpscounter == 16'h0000) begin
                            bpscounter <= PERIOD;
                            bitcnt <= 3'd7;
                            state <= BIT;
                        end
                    end
                BIT:
                    begin
                        tx <= txdata_reg[0];
                        bpscounter <= bpscounter - 16'd1;
                        if (bpscounter == 16'h0000) begin
                            txdata_reg <= {1'b0, txdata_reg[7:1]};
                            bpscounter <= PERIOD;
                            bitcnt <= bitcnt - 3'd1;
                            if (bitcnt == 3'd0) begin
                                state <= STOP;
                            end
                        end
                    end
                STOP:
                    begin
                        tx <= 1'b1;
                        bpscounter <= bpscounter - 16'd1;
                        if (bpscounter == 16'h0000) begin
                            bpscounter <= PERIOD;
                            txbusy_ff <= 1'b0;
                            state <= IDLE;
                        end
                    end
                default:
                    begin
                        state <= IDLE;
                        txbusy_ff <= 1'b0;
                    end
            endcase
        end
    end
endmodule

module uart_rx (
    // CPU interface
    input wire clk,
    output reg [7:0] rxdata,
    output reg rxrecv,
    // RS232 interface
    input wire rx
    );

    initial rxrecv = 1'b0;

    parameter CLK = 28000000;
    parameter BPS = 115200;
    localparam PERIOD = CLK / BPS;
    localparam HALFPERIOD = PERIOD / 2;

    localparam
        IDLE  = 2'd0,
        START = 2'd1,
        BIT   = 2'd2,
        STOP  = 2'd3;

    // Sincronizacin de se�ales externas
    reg [1:0] rx_ff = 2'b00;
    always @(posedge clk) begin
        rx_ff <= {rx_ff[0], rx};
    end

    wire rx_is_1    = (rx_ff == 2'b11);
    wire rx_is_0    = (rx_ff == 2'b00);
    wire rx_negedge = (rx_ff == 2'b10);

    reg [31:0] bpscounter;
    reg [1:0] state = IDLE;
    reg [2:0] bitcnt;

    reg [7:0] rxshiftreg;

    always @(posedge clk) begin
        case (state)
            IDLE:
                begin
                    rxrecv <= 1'b0;   // si estamos aqui, es porque no hay bytes pendientes de leer
                    if (rx_negedge) begin
                        bpscounter <= PERIOD - 2;  // porque ya hemos perdido 2 ciclos detectando el flanco negativo
                        state <= START;
                    end
                end
            START:
                begin
                    bpscounter <= bpscounter - 16'd1;
                    if (bpscounter == HALFPERIOD) begin   // sampleamos el bit a mitad de ciclo
                        if (!rx_is_0) begin  // si no era una se�al de START de verdad
                            state <= IDLE;
                        end
                    end
                    else if (bpscounter == 16'h0000) begin
                        bpscounter <= PERIOD;
                        rxshiftreg <= 8'h00;    // aqui iremos guardando los bits recibidos
                        bitcnt <= 3'd7;
                        state <= BIT;
                    end
                end
            BIT:
                begin
                    bpscounter <= bpscounter - 16'd1;
                    if (bpscounter == HALFPERIOD) begin   // sampleamos el bit a mitad de ciclo
                        if (rx_is_1) begin
                            rxshiftreg <= {1'b1, rxshiftreg[7:1]};   // los bits entran por la izquierda, del LSb al MSb
                        end
                        else if (rx_is_0) begin
                            rxshiftreg <= {1'b0, rxshiftreg[7:1]};
                        end
                        else begin
                            state <= IDLE;
                        end
                    end
                    else if (bpscounter == 16'h0000) begin
                        bitcnt <= bitcnt - 3'd1;
                        bpscounter <= PERIOD;
                        if (bitcnt == 3'd0)
                            state <= STOP;
                    end
                end

            STOP:
                begin
                    bpscounter <= bpscounter - 16'd1;
                    if (bpscounter == HALFPERIOD) begin
                        if (!rx_is_1) begin  // si no era una se�al de STOP de verdad
                            state <= IDLE;
                        end
                        else begin
                            rxrecv <= 1'b1;
                            rxdata <= rxshiftreg;
                            state <= IDLE;
                        end
                    end
                end
            default: state <= IDLE;
        endcase
    end
endmodule


module zxunoregs (
   input clk,
   input rst_n,
   input [15:0] addr,
   input nIORQ,
   input nRD,
   input nWR,
   input [7:0] din,
   output reg [7:0] dout,
   output reg oe,
   output reg [7:0] regaddr,
   output read_from_reg,
   output write_to_reg,
   output regaddr_changed
   );

    // ZXUNO address/data I/O ports for indirect access to ZXUNO registers
    localparam
        IOADDR = 16'hFC3B,
        IODATA = 16'hFD3B;

    reg rregaddr_changed = 1'b0;
    assign regaddr_changed = rregaddr_changed;

    always @(posedge clk) begin
        if (!rst_n) begin
            regaddr <= 8'h00;
            rregaddr_changed <= 1'b1;
        end
        else begin
            if (!nIORQ && addr==IOADDR && !nWR) begin
                regaddr <= din;
                rregaddr_changed <= 1'b1;
            end
            else
                rregaddr_changed <= 1'b0;
        end
    end

    always @* begin
        dout = regaddr;
        if (nIORQ == 1'b0 && addr==IOADDR && nRD == 1'b0)
            oe = 1'b1;
        else
            oe = 1'b0;
    end

    assign read_from_reg = (addr==IODATA && nIORQ == 1'b0 && nRD == 1'b0);
    assign write_to_reg = (addr==IODATA && nIORQ == 1'b0 && nWR == 1'b0);
endmodule


module unouart(
    input         clk,
    input         rst_n,

    // CPU interface
    input         nWR,
    input         nRD,
    input         nIORQ,
    input  [15:0] addr,
    input   [7:0] din,
    output  [7:0] dout,
    output        oe,

    input wire uart_rx,
    output wire uart_tx
    );

    parameter CLK = 28000000;
    parameter BPS = 115200;
    wire txbusy;
    wire data_received;
    wire [7:0] rxdata;
    reg comenzar_trans = 1'b0;
    uart #(.CLK(CLK), .BPS(BPS)) uartchip (
        .clk(clk),
        .txdata(din),
        .txbegin(comenzar_trans),
        .txbusy(txbusy),
        .rxdata(rxdata),
        .rxrecv(data_received),
        .rx(uart_rx),
        .tx(uart_tx)
    );

    reg data_read0 = 0;
    always @(posedge clk)
        data_read0 <= data_read;

    wire data_read;
    wire fifo_is_empty;
    wire [7:0] fifo_q;
    scfifo
    #(
        .lpm_width(8),
        .lpm_widthu(13),
        .lpm_numwords(8192),
        .lpm_showahead("ON"),
        .overflow_checking("ON"),
        .underflow_checking("ON"),
        .add_ram_output_register("ON")
    )
    unouart_fifo (
        .clock(clk),
        .sclr(~rst_n),
        .data(rxdata),
        .rdreq(!data_read && data_read0),
        .wrreq(data_received),
        .empty(fifo_is_empty),
        .q(fifo_q)
    );

    wire [7:0] zxuno_addr;
    wire [7:0] zxunoregs_dout;
    wire zxuno_regrd;
    wire zxuno_regwr;
    wire zxunoregs_oe;
    zxunoregs zxunoregs0 (
        .clk(clk),
        .rst_n(rst_n),
        .nIORQ(nIORQ),
        .nRD(nRD),
        .nWR(nWR),
        .addr(addr),
        .din(din),
        .dout(zxunoregs_dout),
        .oe(zxunoregs_oe),
        .regaddr(zxuno_addr),
        .read_from_reg(zxuno_regrd),
        .write_to_reg(zxuno_regwr)
    );

    // ZXUNO registers for UART (wifi module) handling
    localparam  UARTDATA = 8'hC6,
                UARTSTAT = 8'hC7;

    assign data_read = (zxuno_addr == UARTDATA && zxuno_regrd == 1'b1);

    reg uart_oe;
    reg [7:0] uart_dout;
    always @* begin
        uart_oe = 1'b0;
        uart_dout = 8'hFF;
        if (data_read) begin
            uart_dout = fifo_q;
            uart_oe = 1'b1;
        end
        else if (zxuno_addr == UARTSTAT && zxuno_regrd == 1'b1) begin
            uart_dout = {~fifo_is_empty, txbusy, 6'h00};
            uart_oe = 1'b1;
        end
    end

    always @(posedge clk) begin
        if (zxuno_addr == UARTDATA && zxuno_regwr == 1'b1 && comenzar_trans == 1'b0 && txbusy == 1'b0) begin
            comenzar_trans <= 1'b1;
        end
        if (comenzar_trans == 1'b1 && txbusy == 1'b1) begin
            comenzar_trans <= 1'b0;
        end
    end

    assign oe = zxunoregs_oe | uart_oe;
    assign dout = zxunoregs_oe? zxunoregs_dout : uart_dout;
endmodule
