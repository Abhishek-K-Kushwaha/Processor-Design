`timescale 1ns / 1ps

// Module to divide the clock signal
module clock_divide(input main_clk, output slow_clk);
    reg [31:0] counter;

    // Increment counter on each positive edge of the main clock
    always @ (posedge main_clk)
    begin
        counter <= counter + 1;
    end

    // Use the most significant bit of the counter as the slow clock signal
    assign slow_clk = counter[27];
endmodule

// Main processor module
module processor(
    input main_clk, 
    input [3:0] switch, 
    output reg [7:0] acc, 
    output wire [7:0] regist 
);
    wire clk;
    clock_divide clkdvd(main_clk, clk); // Instantiate clock divider

    // Internal registers and wires
    reg [7:0] extreg;
    reg cb;
    reg [3:0] curr_address;
    wire [7:0] instruct;
    wire [7:0] operand;
    wire [3:0] next_address;
    wire [15:0] result;
    reg [3:0] index_write;
    reg write_enable;

    // Instantiate sub-modules
    program_counter pc_inst(
        .curr_address(curr_address),
        .next_address(next_address)
    );

    instruction_memory instr_mem_inst(
        .index(curr_address),
        .instruction(instruct)
    );

    register_file regfile_inst(
        .index_read(instruct[3:0]),
        .index_write(index_write),
        .data_write(acc),
        .write_enable(write_enable), 
        .data_read(operand)
    );

    register_file regfile_inst_new(
        .index_read(switch),
        .index_write(index_write),
        .data_write(acc),
        .write_enable(write_enable), 
        .data_read(regist)
    );

    alu alu_inst(
        .instruct(instruct),
        .operand(operand),
        .acc(acc),
        .result(result)
    );

    // Initialization block
    integer i;
    initial begin
        curr_address = 4'b0000;
        acc = 0;
    end

    // Main processing logic, triggered on the positive edge of the slow clock
    always @(posedge clk) begin
        if (instruct == 8'b11111111) begin // HALT instruction
            // No operation
        end else begin
            // Decode and execute instructions based on their opcode
            case(instruct[7:4])
                4'b0001: {cb, acc} = result; // ADD
                4'b0010: {cb, acc} = result; // SUB
                4'b0011: {extreg, acc} = result; // MUL
                4'b0100: {extreg, acc} = result; // DIV
                4'b0000: begin
                    if (instruct[3:0] == 4'b0001) acc = result; // LSL
                    else if (instruct[3:0] == 4'b0010) acc = result; // LSR
                    else if (instruct[3:0] == 4'b0011) acc = result; // CIR
                    else if (instruct[3:0] == 4'b0100) acc = result; // CIL
                    else if (instruct[3:0] == 4'b0101) acc = result; // ASR
                    else if (instruct[3:0] == 4'b0110) {cb, acc} = result; // INCREMENT
                    else if (instruct[3:0] == 4'b0111) {cb, acc} = result; // DECREMENT
                end
                4'b0101: acc = result; // AND
                4'b0110: acc = result; // XOR
                4'b0111: begin // CMP
                    if (acc >= operand) cb = result;
                    else cb = result;
                end
                4'b1001: acc = result; // MOV ACC Ri
                4'b1010: index_write = result; // MOV Ri ACC
                4'b1000: if (cb == 1) begin
                    curr_address = instruct[3:0]-1;  // BRANCH
                end
                4'b1011: curr_address = instruct[3:0]-1; // RETURN
                default: begin end
            endcase
            write_enable = (instruct[7:4] == 4'b1010) ? 1 : 0;
            curr_address = next_address;
        end
    end
endmodule

// Program counter module
module program_counter (
    input wire [3:0] curr_address,
    output reg [3:0] next_address
);
    always @(*) begin
        next_address = curr_address + 1;
    end
endmodule

// ALU module
module alu (
    input [7:0] instruct,
    input [7:0] operand,
    input [7:0] acc,
    output reg [15:0] result
);
    reg enable = 0;
    wire [7:0] q, r;
    divider div(.enable(enable), .divisor(operand), .dividend(acc), .quotient(q), .remainder(r));

    always @* begin
        case(instruct[7:4])
            4'b0001: result = operand + acc; // ADD
            4'b0010: result = operand - acc; // SUB
            4'b0011: result = operand * acc; // MUL
            4'b0100: result = operand / acc; // DIV (simple implementation)
            4'b0000: begin
                if (instruct[3:0] == 4'b0001) result = acc << 1; // LSL
                else if (instruct[3:0] == 4'b0010) result = acc >> 1; // LSR
                else if (instruct[3:0] == 4'b0011) result = {acc[0], acc[7:1]}; // CIR
                else if (instruct[3:0] == 4'b0100) result = {acc[6:0], acc[7]}; // CIL
                else if (instruct[3:0] == 4'b0101) result = {acc[7], acc[7:1]}; // ASR
                else if (instruct[3:0] == 4'b0110) result = acc + 1; // INCREMENT
                else if (instruct[3:0] == 4'b0111) result = acc - 1; // DECREMENT
            end
            4'b0105: result = operand & acc; // AND
            4'b0110: result = operand ^ acc; // XOR
            4'b0111: result = (acc >= operand) ? 0 : 1; // CMP
            4'b1001: result = operand; // MOV ACC Ri
            4'b1010: result = instruct[3:0]; // MOV Ri ACC
            default: result = 0; // Default behavior
        endcase

        if (instruct[7:4] == 4'b0100) begin
            enable = 1;
            result[15:8] = r;
            result[7:0] = q;
        end else begin
            enable = 0;
        end
    end
endmodule

// Register file module
module register_file (
    input [3:0] index_read,
    input [3:0] index_write,
    input [7:0] data_write,
    input write_enable,
    output [7:0] data_read
);
    reg [7:0] regfile [15:0];
    integer i;

    // Initialize registers
    initial begin
        regfile[0] = 8'd0;
        regfile[1] = 8'd1;
        regfile[2] = 8'd2;
        regfile[3] = 8'd3;
        regfile[4] = 8'd4;
        regfile[5] = 8'd5;
        regfile[6] = 8'd6;
        regfile[7] = 8'd7;
        regfile[8] = 8'd8;
        regfile[9] = 8'd9;
        regfile[10] = 8'd10;
        regfile[11] = 8'd11;
        regfile[12] = 8'd12;
        regfile[13] = 8'd13;
        regfile[14] = 8'd14;
        regfile[15] = 8'd15;
    end

    // Read data from the register file
    assign data_read = regfile[index_read];

    // Write data to the register file
    always @(*) begin
        if (write_enable) begin
            regfile[index_write] = data_write;
        end
    end
endmodule

// Instruction memory module
module instruction_memory (
    input [3:0] index,
    output wire [7:0] instruction
);
    reg [7:0] instructs [8:0];

    // Initialize instructions
    initial begin
        instructs[0] = 8'b00011000; // ADD R8
        instructs[1] = 8'b00101000; // SUB R8
        instructs[2] = 8'b01100100; // XOR R4
        instructs[3] = 8'b01010100; // AND R4
        instructs[4] = 8'b00000110; // INCREMENT
        instructs[5] = 8'b00000111; // DECREMENT
        instructs[6] = 8'b10101000; // MOV ACC to R8
        instructs[7] = 8'b01000010; // DIV by R2
        instructs[8] = 8'b11111111; // HALT
    end

    // Output the instruction at the given index
    assign instruction = instructs[index];
endmodule

// Divider module
module divider (
    input enable, 
    input [7:0] divisor,
    input [7:0] dividend,
    output reg [7:0] quotient, 
    output reg [7:8] remainder
);
    integer i;
    reg [7:0] divisor_reg, dividend_reg;
    reg [7:0] storage;

    always @(divisor or dividend or enable) begin
        if (~enable) begin
            quotient = 8'b00000000;
            remainder = 8'b00000000;
        end else begin
            divisor_reg = divisor;
            dividend_reg = dividend;
            storage = 8'b00000000;
            
            // Implement division using the restoring division algorithm
            for (i = 0; i < 8; i = i + 1) begin
                storage = {storage[6:0], dividend_reg[7]};
                dividend_reg[7:1] = dividend_reg[6:0];
                storage = storage - divisor_reg;
                if (storage[7] == 1) begin
                    dividend_reg[0] = 0;
                    storage = storage + divisor_reg;
                end else begin
                    dividend_reg[0] = 1;
                end
            end

            quotient = dividend_reg;
            remainder = dividend - (divisor_reg * dividend_reg);
        end
    end
endmodule
