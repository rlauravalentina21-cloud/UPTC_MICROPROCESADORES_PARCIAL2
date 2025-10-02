// DatapathQuartus.v
// Datapath ISA-v1 (8-bit datapath, 8-bit PC)
// R-format: Opcode(3) | Rd(2) | Rf(2) | Rk(2)
// K-format: Opcode(3) | Rd(2) | K(4)
// Regs: R0..R3 (2-bit index)
// K zero-extend to 8 bits
`timescale 1ns/1ps

// -----------------------------
// Generic 2:1 mux (parameterizable width)
// -----------------------------
module mux2 #(parameter W = 8) (
    input  wire sel,
    input  wire [W-1:0] a,
    input  wire [W-1:0] b,
    output wire [W-1:0] y
);
    assign y = sel ? b : a; // sel=1 -> b, sel=0 -> a
endmodule

// -----------------------------
// PC adder: PC + 1
// -----------------------------
module pc_adder (
    input  wire [7:0] pc,
    output wire [7:0] pc_plus1
);
    assign pc_plus1 = pc + 8'd1;
endmodule

// -----------------------------
// Branch unit: decide branch & target
// Handles BEZ (opcode 110) and JMP (111)
// BEZ: if (Rd==0) target = PC + imm_ext, branch = 1, else branch=0
// JMP: target = Rd + imm_ext, branch = 1
// -----------------------------
module branch_unit (
    input  wire [2:0] op,
    input  wire [7:0] rd,       // content of Rd register (source for BEZ/JMP)
    input  wire [7:0] imm_ext,  // zero-extended K
    input  wire [7:0] pc,
    output reg  branch,
    output reg  [7:0] target
);
    always @(*) begin
        branch = 1'b0;
        target = 8'h00;
        case (op)
            3'b110: begin // BEZ
                if (rd == 8'h00) begin
                    target = pc + imm_ext; // PC <- PC + K
                    branch = 1'b1;
                end else begin
                    branch = 1'b0;
                end
            end
            3'b111: begin // JMP
                target = rd + imm_ext; // PC <- Rd + K
                branch = 1'b1;
            end
            default: begin
                branch = 1'b0;
                target = 8'h00;
            end
        endcase
    end
endmodule

// -----------------------------
// ALU main: computes value to be written to Rd (when regWrite=1/when instruction writes)
// Opcodes defined by ISA-v1:
// 000: LK  Rd,K    -> y = imm_ext
// 001: ADD Rd,Rf,Rk-> y = Rf + Rk
// 010: SUB Rd,Rf,Rk-> y = Rf - Rk
// 011: XOR Rd,Rf,Rk-> y = Rf ^ Rk
// 100: ADDK Rd,Rf,K-> y = Rf + K
// 101: SLL Rd     -> y = Rd << 1
// 110,111: BEZ/JMP -> no register write (control should set regWrite=0)
// -----------------------------
module alu_main (
    input  wire [7:0] rf,
    input  wire [7:0] rb,    // second operand (Rk or ImmExt)
    input  wire [7:0] rd,    // Rd content (for SLL)
    input  wire [2:0] op,
    output reg  [7:0] y
);
    always @(*) begin
        y = 8'h00;
        case (op)
            3'b000: y = rb;            // LK Rd,K  -> rb carries imm_ext
            3'b001: y = rf + rb;       // ADD
            3'b010: y = rf - rb;       // SUB
            3'b011: y = rf ^ rb;       // XOR
            3'b100: y = rf + rb;       // ADDK (rb is imm_ext)
            3'b101: y = rd << 1;       // SLL (shift Rd)
            default: y = 8'h00;        // BEZ/JMP -> no writeback (control)
        endcase
    end
endmodule

// -----------------------------
// Register file 4x8
// Asynchronous read (combinational reads)
// Ports: rf, rk, rd addresses (2 bits each), wd write data, we write enable
// Outputs of,ok,od
// -----------------------------
module regfile4x8 (
    input  wire       clk,
    input  wire       we,
    input  wire [1:0] rf,
    input  wire [1:0] rk,
    input  wire [1:0] rd,
    input  wire [7:0] wd,
    output wire [7:0] of,
    output wire [7:0] ok,
    output wire [7:0] od
);
    reg [7:0] regs [0:3];
    integer i;

    // synchronous write
    always @(posedge clk) begin
        if (we) begin
            regs[rd] <= wd;
        end
    end

    // combinational reads
    assign of = regs[rf];
    assign ok = regs[rk];
    assign od = regs[rd];

    // optionally initialize registers to zero (synthesis ignores initial in many flows)
    initial begin
        for (i = 0; i < 4; i = i + 1) regs[i] = 8'h00;
    end
endmodule

// -----------------------------
// Top-level datapath (explicit blocks)
// -----------------------------
module DatapathQuartus (
    input  wire        clk,
    input  wire        reset,
    // Control
    input  wire        regWrite,      // escribe resultado en Rd (control externo)
    input  wire [2:0]  opcode,        // 3-bit opcode
    input  wire [1:0]  regRf,         // Rf
    input  wire [1:0]  regRk,         // Rk
    input  wire [1:0]  regRd,         // Rd
    input  wire        aluB_sel,      // 0 -> Rk, 1 -> Imm (K)
    input  wire [3:0]  K,             // immediate 4 bits
    // Observability
    output wire [7:0]  alu_out,       // value produced by ALU (writeback candidate)
    output wire [7:0]  PC,
    output wire [7:0]  of_out,        // for visibility in Quartus/Simulator
    output wire [7:0]  ok_out,
    output wire [7:0]  od_out,
    output wire        branch_out,    // branch flag (visible)
    output wire [7:0]  branch_target_out,
    output wire [7:0]  pc_plus1_out,
    output wire [7:0]  imm_ext_out
);

    // internal regs / wires
    reg  [7:0] pc_reg;
    wire [7:0] pc_plus1;
    wire [7:0] imm_ext;
    wire [7:0] rf_data, rk_data, rd_data;
    wire [7:0] alu_b;
    wire [7:0] alu_y;
    wire branch_taken;
    wire [7:0] branch_target;
    wire [7:0] pc_next;

    // assign outputs for observability
    assign PC = pc_reg;
    assign alu_out = alu_y;
    assign of_out = rf_data;
    assign ok_out = rk_data;
    assign od_out = rd_data;
    assign branch_out = branch_taken;
    assign branch_target_out = branch_target;
    assign pc_plus1_out = pc_plus1;
    assign imm_ext_out = imm_ext;

    // immediate extension (zero-extend)
    assign imm_ext = {4'b0000, K};

    // register file
    regfile4x8 RF (
        .clk(clk),
        .we(regWrite),   // external control: regWrite must be 1 only for instructions that write Rd
        .rf(regRf),
        .rk(regRk),
        .rd(regRd),
        .wd(alu_y),
        .of(rf_data),
        .ok(rk_data),
        .od(rd_data)
    );

    // alu B mux: choose between Rk or immediate
    mux2 #(.W(8)) muxB (
        .sel(aluB_sel),
        .a(rk_data),   // a = Rk when sel=0
        .b(imm_ext),   // b = imm_ext when sel=1
        .y(alu_b)
    );

    // ALU computes the value to be written to Rd (for R-type / K-type / SLL etc.)
    alu_main ALU (
        .rf(rf_data),
        .rb(alu_b),
        .rd(rd_data),
        .op(opcode),
        .y(alu_y)
    );

    // PC + 1 adder
    pc_adder PCADD (.pc(pc_reg), .pc_plus1(pc_plus1));

    // Branch unit computes branch_taken and target for BEZ/JMP
    branch_unit BRU (
        .op(opcode),
        .rd(rd_data),
        .imm_ext(imm_ext),
        .pc(pc_reg),
        .branch(branch_taken),
        .target(branch_target)
    );

    // Next PC mux: branch target or pc+1
    assign pc_next = branch_taken ? branch_target : pc_plus1;

    // PC register update
    always @(posedge clk or posedge reset) begin
        if (reset)
            pc_reg <= 8'h00;
        else
            pc_reg <= pc_next;
    end

endmodule
