// 
// Copyright (c) 2014-2024 by SHENZHEN PANGO MICROSYSTEMS CO.,LTD
// ALL RIGHTS RESERVED.
// 
// THE SOURCE CODE CONTAINED HEREIN IS PROPRIETARY TO PANGO MICROSYSTEMS, INC.
// IT SHALL NOT BE REPRODUCED OR DISCLOSED IN WHOLE OR IN PART OR USED BY
// PARTIES WITHOUT WRITTEN AUTHORIZATION FROM THE OWNER.
//

// This file implement some GTP cell using yosys internal cell type($reduce_and, $lut, etc...),
//to enable yosys SAT could check equivalence of before_map and after_map 


module GTP_LUT1
#(
    parameter [1:0] INIT = 2'h0
) (
    output wire Z,
    input wire I0
);
   $lut #(.WIDTH(1),.LUT(INIT)) lut1_cell(.A(I0),.Y(Z));
endmodule


module GTP_LUT2
#(
    parameter [3:0] INIT = 4'h0
) (
    output wire Z,
    input wire I0, I1
);
$lut #(.WIDTH(2),.LUT(INIT)) lut1_cell(.A({I1,I0}),.Y(Z));
endmodule



module GTP_LUT3
#(
    parameter [7:0] INIT = 8'h00
) (
    output wire Z,
    input wire I0, I1, I2
);
$lut #(.WIDTH(3),.LUT(INIT)) lut1_cell(.A({I2,I1,I0}),.Y(Z));
endmodule



module GTP_LUT4
#(
    parameter [15:0] INIT = 16'h0000
) (
    output wire Z,
    input wire I0, I1, I2, I3
);
$lut #(.WIDTH(4),.LUT(INIT)) lut1_cell(.A({I3,I2,I1,I0}),.Y(Z));
endmodule


module GTP_LUT5
#(
    parameter [31:0] INIT = 32'h0000_0000
) (
    output wire Z,
    input wire I0, I1, I2, I3, I4
);
$lut #(.WIDTH(5),.LUT(INIT)) lut1_cell(.A({I4,I3,I2,I1,I0}),.Y(Z));
endmodule

module GTP_LUT6
#(
    parameter [63:0] INIT = 64'h0000_0000_0000_0000
) (
    output wire Z,
    input wire I0, I1, I2, I3, I4, I5
);

$lut #(.WIDTH(6),.LUT(INIT)) lut1_cell(.A({I5,I4,I3,I2,I1,I0}),.Y(Z));
endmodule


module GTP_LUT6D
#(
    parameter [63:0] INIT = 64'h0000_0000_0000_0000
) (
    output Z,
    output Z5,
    input I0,
    input I1,
    input I2,
    input I3,
    input I4,
    input I5
);

wire z5a,z5b;
$lut #(.WIDTH(5),.LUT(INIT[31:0])) luta_cell(.A({I4,I3,I2,I1,I0}),.Y(z5a));
$lut #(.WIDTH(5),.LUT(INIT[63:32])) lutb_cell(.A({I4,I3,I2,I1,I0}),.Y(z5b));
$mux #(.WIDTH(1)) u1(.A(z5a),.B(z5b),.S(I5),.Y(Z));
assign Z5 = z5a;
endmodule

