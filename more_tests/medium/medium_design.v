/*
 * medium_design.v
 * A procedurally generated medium-sized test case (around 70 logic gates).
 * Contains a mix of GTP primitives (DFFs) and a random combinational cloud
 * of generic gates ($_) to test the robustness and performance of the mapper/stitcher.
 */
module medium_design(
    input clk,
    input rst,
    input [7:0] data_in,
    output [7:0] data_out
);

    // --- Wires ---
    wire nt_clk, nt_rst;
    wire [7:0] nt_data_in;
    wire [7:0] dff_q;
    wire [7:0] combo_out;
    
    // A large pool of internal wires for the combinational cloud
    wire [63:0] w;

    // --- Primitives ---

    // 1. IO Buffers
    GTP_INBUF clk_ibuf (.I(clk), .O(nt_clk));
    GTP_INBUF rst_ibuf (.I(rst), .O(nt_rst));
    genvar i;
    generate
        for (i = 0; i < 8; i = i + 1) begin : inbuf_loop
            GTP_INBUF in_ibuf (.I(data_in[i]), .O(nt_data_in[i]));
            GTP_OUTBUF out_obuf (.I(combo_out[i]), .O(data_out[i]));
        end
    endgenerate

    // 2. Flip-Flops to register inputs
    generate
        for (i = 0; i < 8; i = i + 1) begin : dff_loop
            GTP_DFF_RE dff (
                .CE(1'b1),
                .CLK(nt_clk),
                .D(nt_data_in[i]),
                .Q(dff_q[i]),
                .R(nt_rst)
            );
        end
    endgenerate

    // 3. The Random Combinational Logic Cloud (~70 gates)
    // Your mapper should consume all of these.
    
    // Layer 1
    $_AND_ g1_0 (.A(dff_q[0]), .B(dff_q[1]), .Y(w[0]));
    $_OR_  g1_1 (.A(dff_q[2]), .B(dff_q[3]), .Y(w[1]));
    $_XOR_ g1_2 (.A(dff_q[4]), .B(dff_q[5]), .Y(w[2]));
    $_NOT_ g1_3 (.A(dff_q[6]), .Y(w[3]));
    $_MUX_ g1_4 (.A(dff_q[7]), .B(w[0]), .S(dff_q[0]), .Y(w[4]));
    $_AND_ g1_5 (.A(dff_q[3]), .B(dff_q[6]), .Y(w[5]));
    $_XOR_ g1_6 (.A(dff_q[1]), .B(dff_q[4]), .Y(w[6]));
    $_OR_  g1_7 (.A(dff_q[5]), .B(dff_q[2]), .Y(w[7]));
    
    // Layer 2
    $_OR_  g2_0 (.A(w[0]), .B(w[1]), .Y(w[8]));
    $_XOR_ g2_1 (.A(w[2]), .B(w[3]), .Y(w[9]));
    $_MUX_ g2_2 (.A(w[4]), .B(w[5]), .S(dff_q[1]), .Y(w[10]));
    $_AND_ g2_3 (.A(w[6]), .B(w[7]), .Y(w[11]));
    $_NOT_ g2_4 (.A(w[1]), .Y(w[12]));
    $_XOR_ g2_5 (.A(w[3]), .B(w[5]), .Y(w[13]));
    $_AND_ g2_6 (.A(w[7]), .B(w[0]), .Y(w[14]));
    $_MUX_ g2_7 (.A(w[2]), .B(w[6]), .S(dff_q[2]), .Y(w[15]));

    // Layer 3
    $_AND_ g3_0 (.A(w[8]), .B(w[10]), .Y(w[16]));
    $_OR_  g3_1 (.A(w[9]), .B(w[11]), .Y(w[17]));
    $_XOR_ g3_2 (.A(w[12]), .B(w[14]), .Y(w[18]));
    $_MUX_ g3_3 (.A(w[13]), .B(w[15]), .S(w[8]), .Y(w[19]));
    $_NOT_ g3_4 (.A(w[10]), .Y(w[20]));
    $_AND_ g3_5 (.A(w[11]), .B(w[9]), .Y(w[21]));
    $_OR_  g3_6 (.A(w[14]), .B(w[13]), .Y(w[22]));
    $_XOR_ g3_7 (.A(w[15]), .B(w[12]), .Y(w[23]));

    // Layer 4
    $_OR_  g4_0 (.A(w[16]), .B(w[17]), .Y(w[24]));
    $_XOR_ g4_1 (.A(w[18]), .B(w[19]), .Y(w[25]));
    $_MUX_ g4_2 (.A(w[20]), .B(w[21]), .S(w[9]), .Y(w[26]));
    $_AND_ g4_3 (.A(w[22]), .B(w[23]), .Y(w[27]));
    $_OR_  g4_4 (.A(w[17]), .B(w[20]), .Y(w[28]));
    $_AND_ g4_5 (.A(w[19]), .B(w[22]), .Y(w[29]));
    $_NOT_ g4_6 (.A(w[16]), .Y(w[30]));
    $_XOR_ g4_7 (.A(w[21]), .B(w[18]), .Y(w[31]));

    // Layer 5
    $_AND_ g5_0 (.A(w[24]), .B(w[30]), .Y(w[32]));
    $_OR_  g5_1 (.A(w[25]), .B(w[28]), .Y(w[33]));
    $_XOR_ g5_2 (.A(w[26]), .B(w[29]), .Y(w[34]));
    $_MUX_ g5_3 (.A(w[27]), .B(w[31]), .S(w[10]), .Y(w[35]));
    $_AND_ g5_4 (.A(w[28]), .B(w[25]), .Y(w[36]));
    $_OR_  g5_5 (.A(w[30]), .B(w[26]), .Y(w[37]));
    $_NOT_ g5_6 (.A(w[31]), .Y(w[38]));
    $_XOR_ g5_7 (.A(w[24]), .B(w[27]), .Y(w[39]));

    // Layer 6
    $_OR_  g6_0 (.A(w[32]), .B(w[33]), .Y(w[40]));
    $_AND_ g6_1 (.A(w[34]), .B(w[35]), .Y(w[41]));
    $_XOR_ g6_2 (.A(w[36]), .B(w[37]), .Y(w[42]));
    $_MUX_ g6_3 (.A(w[38]), .B(w[39]), .S(w[11]), .Y(w[43]));
    $_NOT_ g6_4 (.A(w[33]), .Y(w[44]));
    $_AND_ g6_5 (.A(w[35]), .B(w[32]), .Y(w[45]));
    $_OR_  g6_6 (.A(w[37]), .B(w[38]), .Y(w[46]));
    $_XOR_ g6_7 (.A(w[39]), .B(w[36]), .Y(w[47]));
    
    // Final Output Logic
    $_MUX_ out_mux0 (.A(w[40]), .B(w[41]), .S(dff_q[3]), .Y(combo_out[0]));
    $_XOR_ out_xor1 (.A(w[42]), .B(w[8]), .Y(combo_out[1]));
    $_AND_ out_and2 (.A(w[43]), .B(w[16]), .Y(combo_out[2]));
    $_OR_  out_or3  (.A(w[44]), .B(w[24]), .Y(combo_out[3]));
    $_NOT_ out_not4 (.A(w[45]), .Y(combo_out[4]));
    $_MUX_ out_mux5 (.A(w[46]), .B(w[32]), .S(dff_q[4]), .Y(combo_out[5]));
    $_XOR_ out_xor6 (.A(w[47]), .B(w[0]), .Y(combo_out[6]));
    $_AND_ out_and7 (.A(w[40]), .B(w[47]), .Y(combo_out[7]));

endmodule