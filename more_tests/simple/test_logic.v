// test_logic.v - A simple test case for the Pango mapper
//
// This netlist includes basic logic gates to test the core
// functionality of the mapper, especially the StateEval and 
// GetCutInit functions.

module test_logic (
    input  a,
    input  b,
    input  c,
    input  d,
    input  sel,
    output out_and,
    output out_or,
    output out_xor,
    output out_mux,
    output out_complex
);

    // Internal wires
    wire w_and, w_or, w_xor, w_not, w_mux;

    // --- Basic Gates ---

    // 1. Simple AND gate (should map to a LUT2)
    // INIT for out = a & b would be 16'h8 (binary 1000)
    assign w_and = a & b;
    assign out_and = w_and;

    // 2. Simple OR gate (should map to a LUT2)
    // INIT for out = a | b would be 16'hE (binary 1110)
    assign w_or = a | b;
    assign out_or = w_or;
    
    // 3. Simple XOR gate (should map to a LUT2)
    // INIT for out = a ^ b would be 16'h6 (binary 0110)
    assign w_xor = a ^ b;
    assign out_xor = w_xor;

    // --- Chained Logic & MUX ---

    // 4. A MUX (should map to a LUT3)
    // Logic: out = sel ? d : c
    // INIT for out = (c & ~sel) | (d & sel) would be 32'hD8 (binary 11011000)
    // Inputs: c, d, sel
    assign w_mux = sel ? d : c;
    assign out_mux = w_mux;

    // --- Complex Cone ---
    // A 4-input logic cone that can be mapped into a single LUT4
    // Logic: out = ( (a & b) | (c & d) ) ^ sel
    
    wire and1, and2, or1;
    
    assign and1 = a & b;
    assign and2 = c & d;
    assign or1 = and1 | and2;
    assign out_complex = or1 ^ sel;

endmodule