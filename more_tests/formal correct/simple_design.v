/*
 * simple_design.v
 * A minimal, design_18-style test case for the Pango mapper.
 * It contains a mix of GTP primitives (DFFs) and generic gates ($_)
 * that the mapper needs to handle correctly.
 */
module simple_design(
    input clk,
    input rst,
    input in1,
    input in2,
    input in3,
    input in4,
    output out1,
    output out2
);

    // --- Wires ---
    // Wires from INBUFs
    wire nt_clk, nt_rst, nt_in1, nt_in2, nt_in3, nt_in4;
    
    // Wires from DFF outputs (Prime Inputs for the logic cloud)
    wire dff1_q, dff2_q;
    
    // Wires for combinational logic
    wire and_out, xor_out;
    
    // Wires to OUTBUFs (Prime Outputs for the logic cloud)
    wire combo_to_out1, combo_to_out2;

    // --- Primitives ---

    // 1. Input Buffers (Like in design_18)
    // Your mapper should not touch these.
    GTP_INBUF clk_ibuf (.I(clk), .O(nt_clk));
    GTP_INBUF rst_ibuf (.I(rst), .O(nt_rst));
    GTP_INBUF in1_ibuf (.I(in1), .O(nt_in1));
    GTP_INBUF in2_ibuf (.I(in2), .O(nt_in2));
    GTP_INBUF in3_ibuf (.I(in3), .O(nt_in3));
    GTP_INBUF in4_ibuf (.I(in4), .O(nt_in4));

    // 2. Flip-Flops (The "islands" your mapper must work around)
    // Your mapper should not touch these.
    GTP_DFF_RE dff1 (
        .CE(1'b1),
        .CLK(nt_clk),
        .D(nt_in1),
        .Q(dff1_q),
        .R(nt_rst)
    );

    GTP_DFF_RE dff2 (
        .CE(1'b1),
        .CLK(nt_clk),
        .D(nt_in2),
        .Q(dff2_q),
        .R(nt_rst)
    );

    // 3. The Combinational Logic Cloud (Your mapper's TARGET!)
    // Your mapper should consume all these '$_' gates and replace them with GTP_LUTs.
    
    // An AND gate fed by a DFF output and a primary input
    $_AND_ and_gate (
        .A(dff1_q), 
        .B(nt_in3), 
        .Y(and_out)
    );
    
    // An XOR gate fed by another DFF output and a primary input
    $_XOR_ xor_gate (
        .A(dff2_q), 
        .B(nt_in4), 
        .Y(xor_out)
    );

    // Logic combining the results
    $_OR_ or_gate (
        .A(and_out), 
        .B(xor_out), 
        .Y(combo_to_out1) // This wire goes to an output
    );

    $_NOT_ not_gate (
        .A(and_out), 
        .Y(combo_to_out2) // This wire goes to another output
    );

    // 4. Output Buffers (Like in design_18)
    // Your mapper should not touch these.
    GTP_OUTBUF out1_obuf (.I(combo_to_out1), .O(out1));
    GTP_OUTBUF out2_obuf (.I(combo_to_out2), .O(out2));

endmodule