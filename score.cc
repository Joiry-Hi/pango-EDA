/*
 *  yosys -- Yosys Open SYnthesis Suite
 *
 *  Copyright (C) 2025  Shenzhen Pango Microsystems Co., Ltd. <marketing@pangomicro.com>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include "kernel/yosys.h"
#include "kernel/rtlil.h"
#include "kernel/sigtools.h"
#include "kernel/modtools.h"
#include "kernel/celltypes.h"
#include "kernel/consteval.h"
#include <queue>
#include <ranges>
#include <string.h>

USING_YOSYS_NAMESPACE
using namespace std;
PRIVATE_NAMESPACE_BEGIN

size_t LUT_SIZE = 6;

int GetCost(Module *after_map, Module *before_map,const char* filename);


bool IsNOT(Cell *cell)
{
	if (cell->type != ID($not) && cell->type != ID($_NOT_))
		return false;
    return true;
}
bool IsAND(Cell *cell)
{
	if (cell->type != ID($and) && cell->type != ID($_AND_))
		return false;
    return true;
}
bool IsOR(Cell *cell)
{
	if (cell->type != ID($or) && cell->type != ID($_OR_))
		return false;
    return true;
}
bool IsXOR(Cell *cell)
{
	if (cell->type != ID($xor) && cell->type != ID($_XOR_))
		return false;
    return true;
}
bool IsMUX(Cell *cell)
{
	if (cell->type != ID($mux) && cell->type != ID($_MUX_))
		return false;
    return true;
}

int IsGTP_LUT(Cell *cell)
{
    const char* type_str = cell->type.c_str();
    if(0 == strncmp(type_str,"\\GTP_LUT", 8))
    {
        if(strlen(cell->type.c_str()) == 8 + 1)
        {
            int size = type_str[8] - '0';
            if((size < 0 || size > 6))
            {
                return 0;
            }
            return size;
        }
    }
    return 0;   
}
bool IsGTP_LUT6D(Cell *cell)
{
    if (cell->type != ID(GTP_LUT6D)) return false;
    return true;
}	
bool IsGTP(Cell *cell)  {
    return cell->type.begins_with("\\GTP_");
}
bool IsCombinationalGate(Cell *cell)
{
	return IsAND(cell) || IsOR(cell) || IsNOT(cell) || IsMUX(cell) || IsXOR(cell);
}
bool IsCombinationalCell(Cell *cell)
{
    return IsAND(cell) || IsOR(cell) ||
            IsNOT(cell) || IsMUX(cell)||
            IsXOR(cell) || IsGTP_LUT(cell) || IsGTP_LUT6D(cell) /*|| 
            0 == strncmp(cell->type.c_str(),"\\GTP_MUX2LUT", 12) ||
            cell->type == ID(GTP_INV) ||
            0 == strncmp(cell->type.c_str(),"\\GTP_ROM",8) ||
									  cell->type == ID(GTP_LUT6CARRY)*/
	  ;
}

/*
specify port dirction of each cell for yosys_celltypes/modwalker to 
delcare the input and output pin name of gtp cell   
after that, using yosys_celltypes.cell_output(cell->type,portname) to get port direction
*/
void SetPangoCellTypes(CellTypes* ct)
{
    ct->setup_type(ID(GTP_ADC_E2), {ID(VA),ID(VAUX),ID(DCLK),ID(DADDR),ID(DEN),ID(SECEN),ID(DWE),ID(DI),ID(CONVST),ID(RST_N),ID(LOADSC_N)}, {ID(DO),ID(DRDY),ID(OVER_TEMP),ID(LOGIC_DONE_A),ID(LOGIC_DONE_B),ID(ADC_CLK_OUT),ID(DMODIFIED),ID(ALARM)}, false); 
    ct->setup_type(ID(GTP_APM_E2), {ID(CIN),ID(CPI),ID(CXI),ID(CXBI),ID(X),ID(XB),ID(Y),ID(Z),ID(MODEIN),ID(MODEY),ID(MODEZ),ID(CLK),ID(CEX1),ID(CEX2),ID(CEX3),ID(CEXB),ID(CEY1),ID(CEY2),ID(CEZ),ID(CEM),ID(CEP),ID(CEPRE),ID(CEMODEIN),ID(CEMODEY),ID(CEMODEZ),ID(RSTX),ID(RSTXB),ID(RSTY),ID(RSTZ),ID(RSTM),ID(RSTP),ID(RSTPRE),ID(RSTMODEIN),ID(RSTMODEY),ID(RSTMODEZ)}, {ID(COUT),ID(CPO),ID(CXO),ID(CXBO),ID(P)}, false); 
    ct->setup_type(ID(GTP_BUF), {ID(I)}, {ID(Z)}, false); 
    ct->setup_type(ID(GTP_CFGCLK), {ID(CLKIN),ID(CE_N)}, {}, false); 
    ct->setup_type(ID(GTP_CLKBUFCE), {ID(CE),ID(CLKIN)}, {ID(CLKOUT)}, false); 
    ct->setup_type(ID(GTP_CLKBUFG), {ID(CLKIN)}, {ID(CLKOUT)}, false); 
    ct->setup_type(ID(GTP_CLKBUFGCE), {ID(CLKIN),ID(CE)}, {ID(CLKOUT)}, false); 
    ct->setup_type(ID(GTP_CLKBUFGMUX), {ID(CLKIN0),ID(CLKIN1),ID(SEL)}, {ID(CLKOUT)}, false); 
    ct->setup_type(ID(GTP_CLKBUFGMUX_E1), {ID(CLKIN0),ID(CLKIN1),ID(SEL),ID(EN)}, {ID(CLKOUT)}, false); 
    ct->setup_type(ID(GTP_CLKBUFGMUX_E2), {ID(CLKIN0),ID(CLKIN1),ID(DETECT_CLK0),ID(DETECT_CLK1),ID(SEL)}, {ID(CLKOUT)}, false); 
    ct->setup_type(ID(GTP_CLKBUFM), {ID(CLKIN)}, {ID(CLKOUT)}, false); 
    ct->setup_type(ID(GTP_CLKBUFMCE), {ID(CLKIN),ID(CE)}, {ID(CLKOUT)}, false); 
    ct->setup_type(ID(GTP_CLKBUFR), {ID(CLKIN)}, {ID(CLKOUT)}, false); 
    ct->setup_type(ID(GTP_CLKBUFX), {ID(CLKIN)}, {ID(CLKOUT)}, false); 
    ct->setup_type(ID(GTP_CLKBUFXCE), {ID(CLKIN),ID(CE)}, {ID(CLKOUT)}, false); 
    ct->setup_type(ID(GTP_CLKPD), {ID(RST),ID(CLK_SAMPLE),ID(CLK_CTRL),ID(CLK_PHY),ID(DONE)}, {ID(FLAG_PD),ID(LOCK)}, false); 
    ct->setup_type(ID(GTP_DDC_E2), {ID(RST),ID(RST_TRAINING_N),ID(CLKA),ID(CLKB),ID(DQSI),ID(DQSIB),ID(DELAY_STEP0),ID(DELAY_STEP1),ID(DELAY_STEP2),ID(DELAY_STEP3),ID(DELAY_STEP4),ID(DQS_GATE_CTRL),ID(GATE_SEL),ID(CLK_GATE_CTRL),ID(CLKA_GATE)}, {ID(WCLK),ID(WCLK_DELAY),ID(DQSI_DELAY),ID(DQSIB_DELAY),ID(DGTS),ID(IFIFO_WADDR),ID(IFIFO_RADDR),ID(READ_VALID),ID(DQS_DRIFT),ID(DRIFT_DETECT_ERR),ID(DQS_DRIFT_STATUS),ID(DQS_SAMPLE)}, false); 
    ct->setup_type(ID(GTP_DDC_E2_DFT), {ID(RST),ID(RST_TRAINING_N),ID(CLKA),ID(CLKB),ID(DQSI),ID(DQSIB),ID(DELAY_STEP0),ID(DELAY_STEP1),ID(DELAY_STEP2),ID(DELAY_STEP3),ID(DELAY_STEP4),ID(DQS_GATE_CTRL),ID(GATE_SEL),ID(CLK_GATE_CTRL),ID(CLKA_GATE)}, {ID(WCLK),ID(WCLK_DELAY),ID(DQSI_DELAY),ID(DQSIB_DELAY),ID(DGTS),ID(IFIFO_WADDR),ID(IFIFO_RADDR),ID(READ_VALID),ID(DQS_DRIFT),ID(DRIFT_DETECT_ERR),ID(DQS_DRIFT_STATUS),ID(DQS_SAMPLE),ID(GATE_HIGHB),ID(GATE_HIGH_LATCHB)}, false); 
    ct->setup_type(ID(GTP_DDC_E3), {ID(RST),ID(RST_TRAINING_N),ID(CLKA),ID(CLKB),ID(DQSI),ID(DQSIB),ID(DELAY_STEP0),ID(DELAY_STEP1),ID(DELAY_STEP2),ID(DELAY_STEP3),ID(DELAY_STEP4),ID(DQS_GATE_CTRL),ID(GATE_SEL),ID(CLK_GATE_CTRL),ID(CLKA_GATE)}, {ID(WCLK),ID(WCLK_DELAY),ID(DQSI_DELAY),ID(DQSIB_DELAY),ID(DGTS),ID(IFIFO_WADDR),ID(IFIFO_RADDR),ID(READ_VALID),ID(DQS_DRIFT),ID(DRIFT_DETECT_ERR),ID(DQS_DRIFT_STATUS),ID(DQS_SAMPLE)}, false); 
    ct->setup_type(ID(GTP_DFF), {ID(D),ID(CLK)}, {ID(Q)}, false); 
    ct->setup_type(ID(GTP_DFF_C), {ID(D),ID(CLK),ID(C)}, {ID(Q)}, false); 
    ct->setup_type(ID(GTP_DFF_CE), {ID(D),ID(CLK),ID(C),ID(CE)}, {ID(Q)}, false); 
    ct->setup_type(ID(GTP_DFF_E), {ID(D),ID(CLK),ID(CE)}, {ID(Q)}, false); 
    ct->setup_type(ID(GTP_DFF_P), {ID(D),ID(CLK),ID(P)}, {ID(Q)}, false); 
    ct->setup_type(ID(GTP_DFF_PE), {ID(D),ID(CLK),ID(P),ID(CE)}, {ID(Q)}, false); 
    ct->setup_type(ID(GTP_DFF_R), {ID(D),ID(CLK),ID(R)}, {ID(Q)}, false); 
    ct->setup_type(ID(GTP_DFF_RE), {ID(D),ID(CLK),ID(R),ID(CE)}, {ID(Q)}, false); 
    ct->setup_type(ID(GTP_DFF_S), {ID(D),ID(CLK),ID(S)}, {ID(Q)}, false); 
    ct->setup_type(ID(GTP_DFF_SE), {ID(D),ID(CLK),ID(S),ID(CE)}, {ID(Q)}, false); 
    ct->setup_type(ID(GTP_DLATCH), {ID(D),ID(G)}, {ID(Q)}, false); 
    ct->setup_type(ID(GTP_DLATCH_C), {ID(D),ID(G),ID(C)}, {ID(Q)}, false); 
    ct->setup_type(ID(GTP_DLATCH_CE), {ID(D),ID(G),ID(C),ID(GE)}, {ID(Q)}, false); 
    ct->setup_type(ID(GTP_DLATCH_E), {ID(D),ID(G),ID(GE)}, {ID(Q)}, false); 
    ct->setup_type(ID(GTP_DLATCH_P), {ID(D),ID(G),ID(P)}, {ID(Q)}, false); 
    ct->setup_type(ID(GTP_DLATCH_PE), {ID(D),ID(G),ID(P),ID(GE)}, {ID(Q)}, false); 
    ct->setup_type(ID(GTP_DLL_E2), {ID(CLKIN),ID(SYS_CLK),ID(PWD),ID(RST),ID(UPDATE_N)}, {ID(DELAY_STEP),ID(DELAY_STEP1),ID(LOCK)}, false); 
    ct->setup_type(ID(GTP_DRM18K_E1), {ID(DIA),ID(DIB),ID(ADDRA),ID(ADDRA_HOLD),ID(ADDRB),ID(ADDRB_HOLD),ID(BWEA),ID(BWEB),ID(CLKA),ID(CLKB),ID(CEA),ID(CEB),ID(WEA),ID(WEB),ID(ORCEA),ID(ORCEB),ID(RSTA),ID(RSTB)}, {ID(DOA),ID(DOB)}, false); 
    ct->setup_type(ID(GTP_DRM36K_E1), {ID(CINA),ID(CINB),ID(DIA),ID(DIB),ID(ADDRA),ID(ADDRA_HOLD),ID(ADDRB),ID(ADDRB_HOLD),ID(CSA),ID(CSB),ID(BWEA),ID(BWEB),ID(CLKA),ID(CLKB),ID(CEA),ID(CEB),ID(WEA),ID(WEB),ID(ORCEA),ID(ORCEB),ID(RSTA),ID(RSTB),ID(INJECT_SBITERR),ID(INJECT_DBITERR)}, {ID(COUTA),ID(COUTB),ID(DOA),ID(DOB),ID(ECC_SBITERR),ID(ECC_DBITERR),ID(ECC_RDADDR),ID(ECC_PARITY)}, false); 
    ct->setup_type(ID(GTP_EFUSECODE), {}, {ID(EFUSE_CODE)}, false); 
    ct->setup_type(ID(GTP_FIFO18K_E1), {ID(DI),ID(WCLK),ID(RCLK),ID(WCE),ID(RCE),ID(ORCE),ID(RST)}, {ID(ALMOST_EMPTY),ID(ALMOST_FULL),ID(EMPTY),ID(FULL),ID(DO)}, false); 
    ct->setup_type(ID(GTP_FIFO36K_E1), {ID(DI),ID(WCLK),ID(RCLK),ID(WCE),ID(RCE),ID(ORCE),ID(RST),ID(INJECT_SBITERR),ID(INJECT_DBITERR)}, {ID(ALMOST_EMPTY),ID(ALMOST_FULL),ID(EMPTY),ID(FULL),ID(DO),ID(ECC_SBITERR),ID(ECC_DBITERR)}, false); 
    ct->setup_type(ID(GTP_GPLL), {ID(CLKIN1),ID(CLKIN2),ID(CLKFB),ID(CLKIN_SEL),ID(DPS_CLK),ID(DPS_EN),ID(DPS_DIR),ID(CLKOUT0_SYN),ID(CLKOUT1_SYN),ID(CLKOUT2_SYN),ID(CLKOUT3_SYN),ID(CLKOUT4_SYN),ID(CLKOUT5_SYN),ID(CLKOUT6_SYN),ID(CLKOUTF_SYN),ID(PLL_PWD),ID(RST),ID(APB_CLK),ID(APB_RST_N),ID(APB_ADDR),ID(APB_SEL),ID(APB_EN),ID(APB_WRITE),ID(APB_WDATA)}, {ID(CLKOUT0),ID(CLKOUT0N),ID(CLKOUT1),ID(CLKOUT1N),ID(CLKOUT2),ID(CLKOUT2N),ID(CLKOUT3),ID(CLKOUT3N),ID(CLKOUT4),ID(CLKOUT5),ID(CLKOUT6),ID(CLKOUTF),ID(CLKOUTFN),ID(LOCK),ID(DPS_DONE),ID(APB_RDATA),ID(APB_READY)}, false); 
    ct->setup_type(ID(GTP_GRS), {ID(GRS_N)}, {}, false); 
    ct->setup_type(ID(GTP_HPIO_VREF), {ID(CODE_VREF0),ID(CODE_VREF1),ID(CODE_VREF2),ID(CODE_VREF3)}, {}, false); 
    ct->setup_type(ID(GTP_IDDR_E1), {ID(D),ID(CLK),ID(CE),ID(RS)}, {ID(Q0),ID(Q1)}, false); 
    ct->setup_type(ID(GTP_INBUF), {ID(I)}, {ID(O)}, false); 
    ct->setup_type(ID(GTP_INBUFDS), {ID(I),ID(IB)}, {ID(O)}, false); 
    ct->setup_type(ID(GTP_INBUFDS_E1), {ID(I),ID(IB)}, {ID(O),ID(OB)}, false); 
    ct->setup_type(ID(GTP_INBUFE), {ID(EN),ID(I)}, {ID(O)}, false); 
    ct->setup_type(ID(GTP_INBUFEDS), {ID(EN),ID(I),ID(IB)}, {ID(O)}, false); 
    ct->setup_type(ID(GTP_INBUFEDS_E1), {ID(EN),ID(I),ID(IB)}, {ID(O),ID(OB)}, false); 
    ct->setup_type(ID(GTP_INBUFG), {ID(I)}, {ID(O)}, false); 
    ct->setup_type(ID(GTP_INBUFGDS), {ID(I),ID(IB)}, {ID(O)}, false); 
    ct->setup_type(ID(GTP_INV), {ID(I)}, {ID(Z)}, false); 
    ct->setup_type(ID(GTP_IOBUF), {ID(I),ID(T)}, {ID(O)}, false); 
    ct->setup_type(ID(GTP_IOBUF_RX_MIPI), {ID(I_LP),ID(IB_LP),ID(T),ID(TB),ID(M)}, {ID(O_LP),ID(OB_LP),ID(O_HS)}, false); 
    ct->setup_type(ID(GTP_IOBUF_TX_MIPI), {ID(I_LP),ID(IB_LP),ID(I_HS),ID(T),ID(TB),ID(M)}, {ID(O_LP),ID(OB_LP)}, false); 
    ct->setup_type(ID(GTP_IOBUFCO), {ID(I),ID(T)}, {ID(O)}, false); 
    ct->setup_type(ID(GTP_IOBUFCO_E1), {ID(I),ID(IB),ID(T),ID(TB)}, {ID(O)}, false); 
    ct->setup_type(ID(GTP_IOBUFDS), {ID(I),ID(T)}, {ID(O)}, false); 
    ct->setup_type(ID(GTP_IOBUFE), {ID(I),ID(EN),ID(T)}, {ID(O)}, false); 
    ct->setup_type(ID(GTP_IOBUFECO), {ID(I),ID(EN),ID(T)}, {ID(O)}, false); 
    ct->setup_type(ID(GTP_IOBUFEDS), {ID(I),ID(EN),ID(T)}, {ID(O)}, false); 
    ct->setup_type(ID(GTP_IOCLKBUF), {ID(CLKIN),ID(DI)}, {ID(CLKOUT)}, false); 
    ct->setup_type(ID(GTP_IOCLKDIV_E2), {ID(CLKIN),ID(RST_N),ID(CE)}, {ID(CLKDIVOUT)}, false); 
    ct->setup_type(ID(GTP_IOCLKDIV_E3), {ID(RST),ID(CLKIN)}, {ID(CLKDIVOUT)}, false); 
    ct->setup_type(ID(GTP_IODELAY_E2), {ID(DI),ID(DELAY_SEL),ID(DELAY_STEP),ID(EN_N)}, {ID(DO)}, false); 
    ct->setup_type(ID(GTP_IPAL_E2), {ID(RST_N),ID(CLK),ID(CS_N),ID(RW_SEL),ID(DI)}, {ID(DO),ID(RBCRC_ERR),ID(RBCRC_VALID),ID(ECC_VALID),ID(ECC_INDEX),ID(SERROR),ID(DERROR),ID(SEU_FRAME_ADDR),ID(SEU_COLUMN_ADDR),ID(SEU_REGION_ADDR),ID(SEU_FRAME_NADDR),ID(SEU_COLUMN_NADDR),ID(SEU_REGION_NADDR),ID(PRCFG_OVER),ID(PRCFG_ERR),ID(DRCFG_OVER),ID(DRCFG_ERR)}, false); 
    ct->setup_type(ID(GTP_KEYRAM), {ID(ERASE_KEY_N)}, {}, false); 
    ct->setup_type(ID(GTP_LUT1), {ID(I0)}, {ID(Z)}, false); 
    ct->setup_type(ID(GTP_LUT2), {ID(I0),ID(I1)}, {ID(Z)}, false); 
    ct->setup_type(ID(GTP_LUT3), {ID(I0),ID(I1),ID(I2)}, {ID(Z)}, false); 
    ct->setup_type(ID(GTP_LUT4), {ID(I0),ID(I1),ID(I2),ID(I3)}, {ID(Z)}, false); 
    ct->setup_type(ID(GTP_LUT5), {ID(I0),ID(I1),ID(I2),ID(I3),ID(I4)}, {ID(Z)}, false); 
    ct->setup_type(ID(GTP_LUT6), {ID(I0),ID(I1),ID(I2),ID(I3),ID(I4),ID(I5)}, {ID(Z)}, false); 
    ct->setup_type(ID(GTP_LUT6CARRY), {ID(CIN),ID(I0),ID(I1),ID(I2),ID(I3),ID(I4),ID(I5)}, {ID(COUT),ID(Z)}, false); 
    ct->setup_type(ID(GTP_LUT6D), {ID(I0),ID(I1),ID(I2),ID(I3),ID(I4),ID(I5)}, {ID(Z),ID(Z5)}, false); 
    ct->setup_type(ID(GTP_LUT7), {ID(I0),ID(I1),ID(I2),ID(I3),ID(I4),ID(I5),ID(I6)}, {ID(Z)}, false); 
    ct->setup_type(ID(GTP_LUT8), {ID(I0),ID(I1),ID(I2),ID(I3),ID(I4),ID(I5),ID(I6),ID(I7)}, {ID(Z)}, false); 
    ct->setup_type(ID(GTP_MUX2LUT7), {ID(I0),ID(I1),ID(S)}, {ID(Z)}, false); 
    ct->setup_type(ID(GTP_MUX2LUT8), {ID(I0),ID(I1),ID(S)}, {ID(Z)}, false); 
    ct->setup_type(ID(GTP_ODDR_E1), {ID(D0),ID(D1),ID(CLK),ID(CE),ID(RS)}, {ID(Q)}, false); 
    ct->setup_type(ID(GTP_ONE), {}, {ID(Z)}, false); 
    ct->setup_type(ID(GTP_OSC_E4), {ID(EN_N)}, {ID(CLKOUT)}, false); 
    ct->setup_type(ID(GTP_OSERDES_E2), {ID(RST),ID(OCE),ID(TCE),ID(OCLKDIV),ID(SERCLK),ID(OCLK),ID(MIPI_CTRL),ID(UPD0_SHIFT),ID(UPD1_SHIFT),ID(OSHIFTIN0),ID(OSHIFTIN1),ID(DI),ID(TI),ID(TBYTE_IN)}, {ID(OSHIFTOUT0),ID(OSHIFTOUT1),ID(TQ),ID(DO),ID(TFB),ID(TERM_FB)}, false); 
    ct->setup_type(ID(GTP_OSERDES_FIFO), {ID(DIN),ID(TIN),ID(RST),ID(EN),ID(WCLK),ID(RCLK)}, {ID(DOUT),ID(TOUT),ID(EMPTY),ID(FULL)}, false); 
    ct->setup_type(ID(GTP_OUTBUF), {ID(I)}, {ID(O)}, false); 
    ct->setup_type(ID(GTP_OUTBUFCO), {ID(I)}, {ID(O),ID(OB)}, false); 
    ct->setup_type(ID(GTP_OUTBUFCO_E1), {ID(I),ID(IB)}, {ID(O),ID(OB)}, false); 
    ct->setup_type(ID(GTP_OUTBUFDS), {ID(I)}, {ID(O),ID(OB)}, false); 
    ct->setup_type(ID(GTP_OUTBUFT), {ID(I),ID(T)}, {ID(O)}, false); 
    ct->setup_type(ID(GTP_OUTBUFTCO), {ID(I),ID(T)}, {ID(O),ID(OB)}, false); 
    ct->setup_type(ID(GTP_OUTBUFTCO_E1), {ID(I),ID(IB),ID(T),ID(TB)}, {ID(O),ID(OB)}, false); 
    ct->setup_type(ID(GTP_OUTBUFTDS), {ID(I),ID(T)}, {ID(O),ID(OB)}, false); 
    ct->setup_type(ID(GTP_PPLL), {ID(CLKIN1),ID(CLKIN2),ID(CLKFB),ID(CLKIN_SEL),ID(CLKOUT0_SYN),ID(CLKOUT1_SYN),ID(CLKOUT2_SYN),ID(CLKOUT3_SYN),ID(CLKOUT4_SYN),ID(CLKOUTPHY_SYN),ID(CLKOUTF_SYN),ID(PLL_PWD),ID(RST),ID(APB_CLK),ID(APB_RST_N),ID(APB_ADDR),ID(APB_SEL),ID(APB_EN),ID(APB_WRITE),ID(APB_WDATA)}, {ID(CLKOUT0),ID(CLKOUT0N),ID(CLKOUT1),ID(CLKOUT1N),ID(CLKOUT2),ID(CLKOUT2N),ID(CLKOUT3),ID(CLKOUT3N),ID(CLKOUT4),ID(CLKOUTPHY),ID(CLKOUTPHYN),ID(CLKOUTF),ID(CLKOUTFN),ID(LOCK),ID(APB_RDATA),ID(APB_READY)}, false); 
    ct->setup_type(ID(GTP_PPLL_DFT), {ID(CLKIN1),ID(CLKIN2),ID(CLKFB),ID(CLKIN_SEL),ID(CLKOUT0_SYN),ID(CLKOUT1_SYN),ID(CLKOUT2_SYN),ID(CLKOUT3_SYN),ID(CLKOUT4_SYN),ID(CLKOUTPHY_SYN),ID(CLKOUTF_SYN),ID(PLL_PWD),ID(RST),ID(APB_CLK),ID(APB_RST_N),ID(APB_ADDR),ID(APB_SEL),ID(APB_EN),ID(APB_WRITE),ID(APB_WDATA)}, {ID(PFDTOP_CLK_TEST),ID(CLKOUT0),ID(CLKOUT0N),ID(CLKOUT1),ID(CLKOUT1N),ID(CLKOUT2),ID(CLKOUT2N),ID(CLKOUT3),ID(CLKOUT3N),ID(CLKOUT4),ID(CLKOUTPHY),ID(CLKOUTPHYN),ID(CLKOUTF),ID(CLKOUTFN),ID(LOCK),ID(APB_RDATA),ID(APB_READY)}, false); 
    ct->setup_type(ID(GTP_RAM32X1DP), {ID(DI),ID(RADDR),ID(WADDR),ID(WCLK),ID(WE)}, {ID(DO)}, false); 
    ct->setup_type(ID(GTP_RAM32X1SP), {ID(DI),ID(ADDR),ID(WCLK),ID(WE)}, {ID(DO)}, false); 
    ct->setup_type(ID(GTP_RAM32X2DP), {ID(DI),ID(RADDR),ID(WADDR),ID(WCLK),ID(WE)}, {ID(DO)}, false); 
    ct->setup_type(ID(GTP_RAM32X2SP), {ID(DI),ID(ADDR),ID(WCLK),ID(WE)}, {ID(DO)}, false); 
    ct->setup_type(ID(GTP_RAM32X2X4), {ID(DI0),ID(DI1),ID(DI2),ID(DI3),ID(ADDR0),ID(ADDR1),ID(ADDR2),ID(ADDR3),ID(WCLK),ID(WE)}, {ID(DO0),ID(DO1),ID(DO2),ID(DO3)}, false); 
    ct->setup_type(ID(GTP_RAM64X1DP), {ID(DI),ID(RADDR),ID(WADDR),ID(WCLK),ID(WE)}, {ID(DO)}, false); 
    ct->setup_type(ID(GTP_RAM64X1SP), {ID(DI),ID(ADDR),ID(WCLK),ID(WE)}, {ID(DO)}, false); 
    ct->setup_type(ID(GTP_RAM64X1X4), {ID(DI0),ID(DI1),ID(DI2),ID(DI3),ID(ADDR0),ID(ADDR1),ID(ADDR2),ID(ADDR3),ID(WCLK),ID(WE)}, {ID(DO0),ID(DO1),ID(DO2),ID(DO3)}, false); 
    ct->setup_type(ID(GTP_RAM128X1DP), {ID(DI),ID(RADDR),ID(WADDR),ID(WCLK),ID(WE)}, {ID(DO)}, false); 
    ct->setup_type(ID(GTP_RAM128X1SP), {ID(DI),ID(ADDR),ID(WCLK),ID(WE)}, {ID(DO)}, false); 
    ct->setup_type(ID(GTP_RAM256X1SP), {ID(DI),ID(ADDR),ID(WCLK),ID(WE)}, {ID(DO)}, false); 
    ct->setup_type(ID(GTP_RES_CAL_E1), {ID(PCODE_IN),ID(NCODE_IN),ID(SAMPLE_IN),ID(EN),ID(CODE_SEL),ID(RST_N)}, {ID(PCODE_OUT),ID(NCODE_OUT),ID(CAL_DONE)}, false); 
    ct->setup_type(ID(GTP_RES_CAL_E1_DFT), {ID(PCODE_IN),ID(NCODE_IN),ID(SAMPLE_IN),ID(EN),ID(CODE_SEL),ID(RST_N)}, {ID(PCODE_OUT),ID(NCODE_OUT),ID(CAL_DONE)}, false); 
    ct->setup_type(ID(GTP_RES_CAL_E2), {ID(PCODE_IN),ID(NCODE_IN),ID(SAMPLE_IN),ID(EN),ID(CODE_SEL),ID(RST_N)}, {ID(PCODE_OUT),ID(NCODE_OUT),ID(CAL_DONE)}, false); 
    ct->setup_type(ID(GTP_ROM32X1), {ID(I0),ID(I1),ID(I2),ID(I3),ID(I4)}, {ID(Z)}, false); 
    ct->setup_type(ID(GTP_ROM32X2), {ID(I0),ID(I1),ID(I2),ID(I3),ID(I4)}, {ID(Z)}, false); 
    ct->setup_type(ID(GTP_ROM64X1), {ID(I0),ID(I1),ID(I2),ID(I3),ID(I4),ID(I5)}, {ID(Z)}, false); 
    ct->setup_type(ID(GTP_ROM128X1), {ID(I0),ID(I1),ID(I2),ID(I3),ID(I4),ID(I5),ID(I6)}, {ID(Z)}, false); 
    ct->setup_type(ID(GTP_ROM256X1), {ID(I0),ID(I1),ID(I2),ID(I3),ID(I4),ID(I5),ID(I6),ID(I7)}, {ID(Z)}, false); 
    ct->setup_type(ID(GTP_START_E1), {ID(CLK),ID(GOE),ID(GRS_N),ID(GWE)}, {ID(WAKEUP_OVER)}, false); 
    ct->setup_type(ID(GTP_UDID), {ID(DI),ID(LOAD),ID(SE),ID(CLK)}, {ID(DO)}, false); 
    ct->setup_type(ID(GTP_ZERO), {}, {ID(Z)}, false); 
}

bool RemoveConstInput(vector<SigBit> &inputs, Const& init)
{
	log_assert(init.size() == 1 << inputs.size());
	for (size_t i = 0; i< inputs.size(); ++i)
	{
		SigBit bit = inputs[i];
		if (bit.is_wire()) {
			continue;
		}
		bool is_vcc = bit.data==State::S1;
		Const ninit;
		for(int j=0; j <init.size(); ++j) {
			size_t mask  = size_t(1<<i);
			if (bool(j&mask) == is_vcc) {
				ninit.append(init.at(j));
			}
		}
		init = ninit;
		inputs.erase(inputs.begin() + i);
		--i;
	}
	return true;
}

// GTP_LUT6D may have dont care input, return care input by parameter 'depend_inputs'
// return all inputs(whatever don't care or not) when obit driver is GTP_LUT
bool GetDependInputs(const dict<SigBit, Cell *> &bit2driver_forchecking, const dict<SigBit, vector<Cell *>> &bit2reader_forchecking,
		      const SigMap& sigmap_forchecking,
				SigBit obit, vector<SigBit>& depend_inputs)
 {
	 if (!bit2driver_forchecking.count(obit)) {
		return false;
	 }
	 Cell *drv = bit2driver_forchecking.at(obit);
	 if (!IsCombinationalCell(drv)) {
		return false;
	 }
	 bool is_gtp_lut6d = IsGTP_LUT6D(drv);
	 IdString output_port_name;
	 vector<SigBit> input_bits;
	 for (auto &conn : drv->connections()) {
		 IdString portname = conn.first;
		 RTLIL::SigSpec sig = sigmap_forchecking(conn.second);
		 if (yosys_celltypes.cell_output(drv->type, portname)) {
			 for (int i = 0; i < sig.size(); i++) {
				 if (sig[i] == obit) {
					output_port_name = portname;
				 }
			 }
			 continue;
		 }
		 if (yosys_celltypes.cell_input(drv->type, portname)) {
			if (sig.size()!=1) {
				 log_warning("cell %s port %s connect %d bit.\n", drv->name.c_str(), portname.c_str(), sig.size()); 
			 }
			 SigBit bit = sig.as_bit();
			 if (!is_gtp_lut6d) {
				 input_bits.push_back(bit);
			 }
			 else {
				 const char *port_name_p = portname.c_str() + 1; //portname is like "\I1"
				 log_assert(*port_name_p == 'I');
				 char num = *(port_name_p + 1) - '0';
				 log_assert(num >= 0 && num < 6);
				 if (size_t(num + 1) >= input_bits.size()) {
					 input_bits.resize(num + 1);
				 }
				 input_bits[num] = bit;
			 }
		 }
	 }
	 if (!output_port_name.size()) {
		 log_error("not found %s on driver cell %s\n",log_signal(obit),drv->name.c_str());
	 }
	 if (!is_gtp_lut6d)
	 {
		 depend_inputs = input_bits;
		 return true;
	 }
	 if (!drv->hasParam(ID::INIT)) {
		 log_error("not found init on driver cell %s %s\n", drv->name.c_str(), log_signal(obit));
		 return false;
	 }
	 Const initval = drv->getParam(ID::INIT);
	 if (initval.size() != 64) {
		 log_error("init on driver cell %s  with size %d\n", drv->name.c_str(), initval.size());
	 }
	 if (output_port_name == ID(Z5))
	 {
		 vector<SigBit> z5inputs(input_bits.begin(),input_bits.begin() + 5);
		 Const z5init = initval.extract(0,32);
		 RemoveConstInput(z5inputs,z5init);
		 for (size_t i = 0; i < z5inputs.size(); i++) {
			 if (!input_bits[i].is_wire()) {
				 continue;
			 }
			 for (int idx = 0; idx < z5init.size(); idx++) {
				 int mask = int(1 << i);
				 int cof_index = idx ^ mask;
				 if (initval.at(idx) != initval.at(cof_index)) {
					 // input_bits[i] is not a don't care input
					 depend_inputs.push_back(input_bits[i]);
					 break;
				 }
			 }
		 }
	 } else if (output_port_name == ID(Z)) {
		 RemoveConstInput(input_bits, initval);
		 for (size_t i = 0; i < input_bits.size(); i++) {
			 if (!input_bits[i].is_wire()) {
				 continue;
			 }
			 for (int idx = 0; idx < initval.size(); idx++) {
				 int mask = int(1 << i);
				 int cof_index = idx ^ mask;
				 if (initval.at(idx) != initval.at(cof_index)) {
					 // input_bits[i] is not a don't care input
					 depend_inputs.push_back(input_bits[i]);
					 break;
				 }
			 }
		 }
	 }
	 else {
		 log_error("unsupport port %s.\n",output_port_name.c_str());
	 }
	 
	 return true;
 };


 int GetCost(Module *after_module, Module *before_module,const char* filename)
 {
    int cost = 0;
    bool map_failed = false;
	int max_level = 0;
    int num_of_luts = 0;
    int num_of_pins = 0;
    for (Cell *cell : before_module->cells())
	{
	    if (!IsGTP(cell) || cell->type == ID(GTP_GRS))
		{
			continue;
		}
	    if (after_module->cells_.count(cell->name) == 0) {
		    map_failed = true; 
		    log_warning("MAP-FAILED due to %s(%s) not found in after module.\n", cell->name.c_str(), cell->type.c_str());
	    }
	    Cell *cell_af = after_module->cells_[cell->name];
		if(cell->type != cell_af->type)
		{
		    map_failed = true;
		    log_warning("MAP-FAILED due to %s(%s) not equal to %s(%s) in after module.\n", cell->name.c_str(), cell->type.c_str(),
				cell->name.c_str(), cell_af->type.c_str());		
		}
		for(auto con : cell->connections())
		{
			IdString portname = con.first;
			SigSpec sig = con.second;
			SigSpec sig_af = cell_af->getPort(portname);
			if (sig.as_string() != sig_af.as_string())
			{
				map_failed = true;
				log_warning("MAP-FAILED due to the net connect to port %s of %s(%s) is changed  {%s} != {%s}.\n", portname.c_str(),
					    cell->name.c_str(), cell->type.c_str(), sig.as_string().c_str(), sig_af.as_string().c_str());
			}
		}
	}
    for (Cell *cell : after_module->cells()) 
    {
	    if (IsCombinationalGate(cell)) 
        {
		    log_warning("MAP-FAILED due to have unmaped cell %s(%s).\n",cell->name.c_str(),cell->type.c_str());
            map_failed = true;
            continue;
        }
		else if (!IsGTP(cell))
		{
			log_warning("MAP-FAILED due to have unknow cell %s(%s).\n", cell->name.c_str(), cell->type.c_str());
			map_failed = true;
			continue;
		}
        int lut_size = IsGTP_LUT(cell);
        if(lut_size > 0)
        {
            map_failed |= (lut_size > int(LUT_SIZE));
			if (lut_size>6) {
		    log_warning("MAP-FAILED due to lut size %s %d > %ld.\n", cell->name.c_str(), lut_size, LUT_SIZE);
			}
            num_of_luts += 1;
            num_of_pins += lut_size;
        }
        else if (IsGTP_LUT6D(cell)) 
        {
            num_of_luts += 1;
            num_of_pins += 6;
        }
		else if(before_module->cells_.count(cell->name) == 0)
		{
			map_failed = true;
			log_warning("MAP-FAILED due to %s(%s) not found in before module.\n", cell->name.c_str(), cell->type.c_str());
		}
		else if (before_module->cells_[cell->name]->type != cell->type)
		{
			map_failed = true;
			log_warning("MAP-FAILED due to %s(%s) not equal to %s(%s) in before module.\n",
			cell->name.c_str(), cell->type.c_str(),
				    cell->name.c_str(),
				    before_module->cells_[cell->name]->type.c_str());		
		}
    }
    
    SigMap sigmap_forchecking(after_module);
    dict<SigBit, Cell*> bit2driver_forchecking;
    dict<SigBit, vector<Cell*>> bit2reader_forchecking;
    for (auto &cell_iter : after_module->cells_)
    {
        Cell* cell= cell_iter.second;
        if(!yosys_celltypes.cell_known(cell->type))
        {   
            continue;
        }
        if(!IsCombinationalCell(cell))
        {
            continue;
        }
        for (auto &conn : cell->connections())
        {
            IdString portname = conn.first;
            RTLIL::SigSpec sig = sigmap_forchecking(conn.second);
            if (yosys_celltypes.cell_output(cell->type, portname))
            {
                for (int i = 0; i < sig.size(); i++)
                {
                    bit2driver_forchecking[sig[i]] = cell;
                }
            }
            else if(yosys_celltypes.cell_input(cell->type, portname))
            {
                for (int i = 0; i < sig.size(); i++)
                {
                    bit2reader_forchecking[sig[i]].push_back(cell);
                }
            }
        }
    }

	dict<SigBit, int> bit_maxDepth;
    pool<SigBit> bit_visited;
    pool<SigBit> bit_visiting;
    function<int(SigBit edge)> BitDFS = [&](SigBit edge) {
	    if (!edge.is_wire() || bit_maxDepth.count(edge)) {
		    return bit_maxDepth[edge];
	    }
	    if (!bit2driver_forchecking.count(edge)) {
		    bit_maxDepth[edge]=0;
			return 0;
		}
	    Cell *node = bit2driver_forchecking[edge];
	    if (!IsCombinationalCell(node)) {
		    bit_maxDepth[edge] = 0;
		    return 0;
	    }
	    if (bit_visiting.count(edge)) {
		    map_failed = true;
		    log_warning("MAP-FAILED due to cycle detected at node: %s  net: %s\n", node->name.c_str(),log_signal(edge));
		    return 0;
		}

	    bit_visiting.insert(edge);
	    int cur_max_level = 0;
	    bool found_obit_on_cell = false;
	    for (auto &conn : node->connections()) {
		    IdString portname = conn.first;
		    RTLIL::SigSpec sig = sigmap_forchecking(conn.second);
		    if (yosys_celltypes.cell_output(node->type, portname)) {
			    for (int i = 0; i < sig.size(); i++) {
				    SigBit obit = sig[i];
				    if (obit != edge) {
						continue;
					}
				    found_obit_on_cell = true;
				    vector<SigBit> depend_inputs;
				    bool ret =
				      GetDependInputs(bit2driver_forchecking, bit2reader_forchecking, sigmap_forchecking, obit, depend_inputs);
				    if (!ret) {
					    log_warning("MAP-FAILED due to get depend inputs of %s on %s.\n", log_signal(obit), node->name.c_str());
					    map_failed = true;
				    }
				    for (SigBit bit : depend_inputs) {
					    cur_max_level = max(cur_max_level, BitDFS(bit) + 1);
				    }
			    }
		    }
			
	    }

	    if (false == found_obit_on_cell) {
		    log_warning("MAP-FAILED not found net %s on driver cell %s\n", log_signal(edge), node->name.c_str());
		    bit_maxDepth[edge] = 0;
		    return cur_max_level;
	    }

	    bit_visiting.erase(edge);
	    bit_visited.insert(edge);
	    bit_maxDepth[edge] = cur_max_level;
	    return cur_max_level;
    };

    for (Cell *cell : after_module->cells())
	{
		if(IsGTP_LUT6D(cell)){
		    vector<SigBit> z_depend_inputs;
			vector<SigBit> z5_depend_inputs;
			SigBit zbit = cell->getPort(ID(Z)).as_bit();
		    SigBit z5bit = cell->getPort(ID(Z5)).as_bit();
		    if (!GetDependInputs(bit2driver_forchecking, bit2reader_forchecking, sigmap_forchecking, zbit, z_depend_inputs)) {
			    log_warning("MAP-FAILED due to get depend inputs of z pin %s on %s.\n", log_signal(zbit), cell->name.c_str());
			    map_failed = true;
		    }
		    if (!GetDependInputs(bit2driver_forchecking, bit2reader_forchecking, sigmap_forchecking, z5bit, z5_depend_inputs)) {
			    log_warning("MAP-FAILED due to get depend inputs of z5 pin %s on %s.\n", log_signal(z5bit), cell->name.c_str());
			    map_failed = true;
		    }
			int share_input_cnt=0;
		    for (SigBit bit : z_depend_inputs) {
				if (find(z5_depend_inputs.begin(),z5_depend_inputs.end(),bit) != z5_depend_inputs.end()) {
				    share_input_cnt++;
				}
		    }
			if(share_input_cnt<1){
			    log_warning("MAP-FAILED due to z and z5 share %d common inputs of %s.\n", share_input_cnt, cell->name.c_str());
			    map_failed = true;
			}
		}
		for (auto &conn : cell->connections()) {
			IdString portname = conn.first;
			RTLIL::SigSpec sig = sigmap_forchecking(conn.second);
			if (yosys_celltypes.cell_output(cell->type, portname)) {
				for (int i = 0; i < sig.size(); i++) {
					SigBit obit = sig[i];
					if (bit_visiting.count(obit)) {
						log_warning("MAP-FAILED bit %s may have loop.\n",log_signal(obit));
					}
					if (bit_visited.count(obit)) {
						max_level = max(max_level, bit_maxDepth[obit]);
						continue;
					}
					vector<SigBit> depend_inputs;
					GetDependInputs(bit2driver_forchecking, bit2reader_forchecking, sigmap_forchecking, obit, depend_inputs);
					for (SigBit bit : depend_inputs) {
						max_level = max(max_level, BitDFS(bit) + 1);
					}
				}
			}
		}
    }
    const char *score_file_name = filename ? filename : "score.txt";
    if (nullptr == filename)
    {
        log("Empty file name is replaced with score.txt.\n");
    }
    ofstream of;
    of.open(score_file_name, ios::out);
	if (!of.is_open()) 
    {
		log_error("can not open file %s.\n", score_file_name);
	}
    if(!map_failed)
    {
        cost = (max_level/20.0 +1)*num_of_luts*10 + num_of_pins;
    }
    of << "cost : " << cost << endl;
    of << "max_level : " << max_level << endl;
    of << "num_of_luts : " << num_of_luts << endl;
    of << "num_of_pins : " << num_of_pins << endl;
	of.close();
    log("write %s.\n", score_file_name);
    return cost;
 }  

struct ScorePass : public ScriptPass {
	 ScorePass() : ScriptPass("score", "get score for mapper result.") {}
	 void help() override
	 {
		 //   |---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|
		 log("\n");
		 log("    score\n");
	 }
	 string before_map_file;
	 string after_map_file;
     string score_file_name = "score.txt";
	 void clear_flags() override
	 {
		before_map_file = "";
		after_map_file = "";
        score_file_name = "score.txt";
	 }
	 void execute(std::vector<std::string> args, RTLIL::Design *design) override
	 {
		 log_header(design, "Start score\n");
		 string run_from, run_to;
		 size_t argidx;
		 for (argidx = 1; argidx < args.size(); argidx++) {
			 if (args[argidx] == "-before" && argidx + 1 < args.size()) {
				 before_map_file = args[++argidx];
				 continue;
			 }
			 if (args[argidx] == "-after" && argidx + 1 < args.size()) {
				 after_map_file = args[++argidx];
				 continue;
			 }
             if (args[argidx] == "-out" && argidx + 1 < args.size()) {
                 score_file_name = args[++argidx];
                 continue;
             }
			 break;
		 }
		 extra_args(args, argidx, design);

		 log_header(design, "Continuing Score pass.\n");
		 SetPangoCellTypes(&yosys_celltypes);
		 run_script(design, run_from, run_to);
		 
		 log_pop();
	 }

	 void script() override
	 {
		 Module *before_map_module = nullptr;
		 Module *after_map_module = nullptr;
		 IdString top_module_name;
		 if (check_label("begin"))
		 {
			 run("read_verilog -lib +/pango/pango_lib.v");
			 run(stringf("read_verilog -overwrite -icells %s", before_map_file.c_str()));
			 before_map_module = active_design->top_module();
			 if (before_map_module == nullptr)
				 log_cmd_error("No top module found in before file.\n");
			 top_module_name = before_map_module->name;
			 active_design->rename(before_map_module, ID(before));

			 run(stringf("read_verilog -overwrite -icells  %s", after_map_file.c_str()));
			 after_map_module = active_design->module(top_module_name);
			 if (after_map_module == nullptr)
				 log_cmd_error("Not found %s module in after_map file.\n", top_module_name.c_str());
			 active_design->rename(after_map_module, ID(after));
		 }

		 if (before_map_module == nullptr || after_map_module == nullptr)
		 {
			 log_cmd_error("nullptr module found.\n");
		 }

		 if (check_label("cost")) {
			 GetCost(after_map_module, before_map_module, score_file_name.c_str());
		 }
	 }
} ScorePass;

PRIVATE_NAMESPACE_END
