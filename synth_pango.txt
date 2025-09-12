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

/*
Area flow based lut-mapper.
Implement of "Heuristics for Area Minimization in LUT-Based FPGA Technology Mapping".
*/

#include "kernel/celltypes.h"
#include "kernel/consteval.h"
#include "kernel/modtools.h"
#include "kernel/sigtools.h"
#include "kernel/yosys.h"
#include <queue>
#include <ranges>
#include <string.h>

USING_YOSYS_NAMESPACE
using namespace std;
PRIVATE_NAMESPACE_BEGIN

// -----------------------
// global variables delare here

size_t MAX_CUT_SIZE_PRE_CELL = 300;
size_t MAX_INTERATIONS = 3;
size_t LUT_SIZE = 6;

dict<SigBit, pool<SigBit>> best_bit2cut;
size_t cur_interation = 0;
// using sigmap to get a unique name of each signal
SigMap sigmap;

// fast get reader or driver by sigbit
dict<SigBit, Cell *> bit2driver;
dict<SigBit, vector<Cell *>> bit2reader;
dict<Cell *, vector<SigBit>> cell2bits; // the first bit is output bits

dict<Cell *, float> cell2OptDepth;
dict<SigBit, float> bit2height;
dict<SigBit, float> bit2depth;
dict<SigBit, float> bit2af;
dict<SigBit, size_t> bit2fanout_est;
dict<Cell *, dict<pool<SigBit>, pool<Cell *>>> cell2cuts; // cell -> dict<cut, cone>

bool using_internel_lut_type = false;
//-------------------------------
// functions declare here
bool MapperMain(Module *module);
bool MapperInit(Module *module);
void SetPangoCellTypes(CellTypes *);
bool CheckCellWidth(Module *module);
bool GetPrimeInputOuput(Module *module, pool<SigBit> &inputs, pool<SigBit> &outputs);
bool GenerateCuts(Module *module);
bool TraverseBWD(Module *module, const pool<SigBit> &, dict<SigBit, pool<SigBit>> &);
bool TraverseFWD(Module *module, const pool<SigBit> &, dict<SigBit, pool<SigBit>> &);
bool ConeToLUTs(Module *module, dict<SigBit, pool<SigBit>> &bit2cut);
float GetEstimatedFanout(SigBit bit);

pool<Cell *> GetReaders(Cell *cell, RTLIL::IdString port = RTLIL::IdString());

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
	const char *type_str = cell->type.c_str();
	if (0 == strncmp(type_str, "\\GTP_LUT", 8)) {
		if (strlen(cell->type.c_str()) == 8 + 1) {
			int size = type_str[8] - '1';
			if ((size < 0 || size > 6)) {
				return 0;
			}
			return size;
		}
	}
	return 0;
}
bool IsGTP_LUT6D(Cell *cell)
{
	if (cell->type != ID(GTP_LUT6D))
		return false;
	return true;
}
bool IsGTP(Cell *cell) { return cell->type.begins_with("\\GTP_"); }
bool IsCombinationalGate(Cell *cell) { return IsAND(cell) || IsOR(cell) || IsNOT(cell) || IsMUX(cell) || IsXOR(cell); }
bool IsCombinationalCell(Cell *cell)
{
	return IsAND(cell) || IsOR(cell) || IsNOT(cell) || IsMUX(cell) || IsXOR(cell) || IsGTP_LUT(cell) || IsGTP_LUT6D(cell);
}

// only return this first sigbit connect to cell
SigBit GetCellOutput(Cell *cell)
{
	log_assert(cell && cell2bits.count(cell));
	auto &bits = cell2bits[cell];
	return bits[0];
}
void GetCellInputsSet(Cell *cell, pool<SigBit> &inputs)
{
	log_assert(cell && inputs.empty() && cell2bits.count(cell));
	auto &bits = cell2bits[cell];
	int offset = 1;
	// not support GTP_LUT6D now
	// if(cell is dual output)
	// offset=2
	for (auto it = bits.begin() + offset; it != bits.end(); ++it) {
		inputs.insert(*it);
	}
}
void GetCellInputsVector(Cell *cell, vector<SigBit> &inputs)
{
	log_assert(cell && inputs.empty() && cell2bits.count(cell));
	auto &bits = cell2bits[cell];
	int offset = 1;
	// not support GTP_LUT6D now
	// if(cell is dual output)
	// offset=2
	for (auto it = bits.begin() + offset; it != bits.end(); ++it) {
		inputs.push_back(*it);
	}
}

pool<Cell *> GetReaders(Cell *cell, RTLIL::IdString port)
{
	pool<Cell *> ret;
	log_assert(cell && !port.empty() && cell->connections().count(port));
	SigSpec sig = cell->getPort(port);
	sig = sigmap(sig);
	for (int i = 0; i < sig.size(); i++) {
		SigBit bit = sig[i];
		vector<Cell *> readers = bit2reader[bit];
		for (size_t i = 0; i < readers.size(); i++) {
			ret.insert(readers[i]);
		}
	}
	return ret;
}

// Primie input and output not means the netlist input/output.
// It is the input of first gate which have no GTP driver,
// or the output which have not combinational gate reader.
bool GetPrimeInputOuput(Module *module, pool<SigBit> &inputs, pool<SigBit> &outputs)
{
	for (auto &cell_iter : module->cells_) {
		Cell *cell = cell_iter.second;
		if (!cell || !IsCombinationalGate(cell)) // only connsider the prime input and output connect to the combinational gate
		{
			continue;
		}
		for (auto &conn : cell->connections()) {
			IdString portname = conn.first;
			RTLIL::SigSpec sig = sigmap(conn.second);
			if (yosys_celltypes.cell_output(cell->type, portname)) {
				pool<Cell *> readers = GetReaders(cell, portname);
				for (Cell *reader : readers) {
					if (!IsCombinationalGate(reader)) {
						outputs.insert(sig[0]);
						break;
					}
				}
				if (readers.size() < 1) {
					outputs.insert(sig[0]);
				}
			} else {
				for (int i = 0; i < sig.size(); i++) {
					SigBit bit = sig[i];
					Cell *drv = bit2driver.count(bit) ? bit2driver[bit] : nullptr;
					if (!drv || !IsCombinationalGate(drv)) {
						inputs.insert(bit);
						break;
					}
				}
			}
		}
	}
	return true;
}

// return all combinational gates in module
void GetGates(Module *module, pool<Cell *> &gates)
{
	gates.clear();
	for (auto &cell_iter : module->cells_) {
		Cell *cell = cell_iter.second;
		if (IsCombinationalGate(cell)) {
			gates.insert(cell);
		}
	}
}

void GetTopoSortedGates(Module *module, vector<Cell *> &gates)
{
	gates.clear();
	pool<SigBit> prime_inputs;
	pool<SigBit> prime_outputs;
	GetPrimeInputOuput(module, prime_inputs, prime_outputs);
	dict<Cell *, size_t> indegree;
	pool<Cell *> visited;
	queue<Cell *> zero_indegree_nodes;
	for (auto &it : cell2bits) {
		Cell *cell = it.first;
		log_assert(it.second.size() > 0);
		indegree[cell] = it.second.size() - 1; // the first is output bit
		if (!IsCombinationalGate(cell)) {
			continue;
		}
		vector<SigBit> inputs;
		GetCellInputsVector(cell, inputs);
		for (auto b : inputs) {
			Cell *drv = bit2driver[b];
			if (!drv || !IsCombinationalGate(drv)) {
				indegree[cell]--;
			}
		}
		if (indegree[cell] == 0) {
			zero_indegree_nodes.push(cell);
		}
	}
	// BFS
	while (!zero_indegree_nodes.empty()) {
		Cell *current = zero_indegree_nodes.front();
		zero_indegree_nodes.pop();
		if (!IsCombinationalGate(current) || visited.find(current) != visited.end()) {
			continue;
		}
		gates.push_back(current);
		visited.insert(current);
		auto rds = GetReaders(current, ID(Y)); // combination gate output pin are Y
		for (Cell *neighbor : rds) {
			if (!IsCombinationalGate(current)) {
				continue;
			}
			if (visited.count(neighbor)) {
				log_error("toposort found loop, cell %s reader %s\n", current->name.c_str(), neighbor->name.c_str());
			}
			if (--indegree[neighbor] == 0) // update indegree
			{
				zero_indegree_nodes.push(neighbor);
			}
		}
	}
}

bool MapperMain(Module *module)
{
	CheckCellWidth(module);
	vector<Cell *> gates;
	GetTopoSortedGates(module, gates);
	GenerateCuts(module);
	pool<SigBit> prime_inputs;
	pool<SigBit> prime_outputs;
	GetPrimeInputOuput(module, prime_inputs, prime_outputs);
	log_debug("found %ld prime input and %ld prime output\n", prime_inputs.size(), prime_outputs.size());
	// init fanout, oedges(v)
	for (auto &p : bit2reader) {
		bit2fanout_est[p.first] = p.second.size();
	}
	dict<SigBit, pool<SigBit>> bit2cut;
	for (cur_interation = 0; cur_interation < MAX_INTERATIONS; cur_interation++) {
		bit2cut.clear();
		TraverseFWD(module, prime_inputs, bit2cut);
		TraverseBWD(module, prime_outputs, bit2cut);
		log_debug("iteration = %ld  cut_num = %ld\n", cur_interation, bit2cut.size());
		if (best_bit2cut.size() == 0 || bit2cut.size() < best_bit2cut.size()) {
			best_bit2cut = bit2cut;
		}

		if (cur_interation == 0) {
			function<void(Cell *, float)> MarkODepth = [&](Cell *cell, float ODepth) {
				if (!cell || !IsCombinationalGate(cell)) {
					return;
				}
				if (cell2OptDepth.count(cell) && cell2OptDepth[cell] >= ODepth) {
					return;
				}
				cell2OptDepth[cell] = ODepth;
				pool<SigBit> inputs;
				GetCellInputsSet(cell, inputs);
				for (auto bit : inputs) {
					Cell *drv = bit2driver.count(bit) ? bit2driver[bit] : nullptr;
					if (!drv) {
						continue;
					}
					MarkODepth(drv, ODepth);
				}
			};
			// the first interation depth is the optimal mapping depth
			for (auto it = gates.rbegin(); it != gates.rend(); ++it) {
				Cell *cell = *it;
				SigBit bit = GetCellOutput(cell);
				Cell *drv = bit2driver.count(bit) ? bit2driver[bit] : nullptr;
				if (!drv || !prime_outputs.count(bit)) {
					continue;
				}
				MarkODepth(drv, bit2depth[bit]);
			}
		}
	}
	log_debug("Map cut to GTP_LUT\n");
	ConeToLUTs(module, best_bit2cut);
	return true;
}

// -----------------------
bool MapperInit(Module *module)
{
	log_debug("Pango init\n");
	SetPangoCellTypes(&yosys_celltypes);

	best_bit2cut.clear();
	cur_interation = 0;
	bit2driver.clear();
	bit2reader.clear();
	cell2bits.clear();

	cell2OptDepth.clear();
	bit2height.clear();
	bit2depth.clear();
	bit2af.clear();
	bit2fanout_est.clear();
	cell2cuts.clear();
	sigmap.set(module);
	return true;
}

/*
specify port dirction of each cell for yosys_celltypes/modwalker to
delcare the input and output pin name of gtp cell
after that, using yosys_celltypes.cell_output(cell->type,portname) to get port direction
*/
void SetPangoCellTypes(CellTypes *ct)
{
	ct->setup_type(ID(GTP_ADC_E2),
		       {ID(VA), ID(VAUX), ID(DCLK), ID(DADDR), ID(DEN), ID(SECEN), ID(DWE), ID(DI), ID(CONVST), ID(RST_N), ID(LOADSC_N)},
		       {ID(DO), ID(DRDY), ID(OVER_TEMP), ID(LOGIC_DONE_A), ID(LOGIC_DONE_B), ID(ADC_CLK_OUT), ID(DMODIFIED), ID(ALARM)}, false);
	ct->setup_type(ID(GTP_APM_E2), {ID(CIN),   ID(CPI),   ID(CXI),	ID(CXBI),  ID(X),	 ID(XB),	ID(Y),	      ID(Z),	   ID(MODEIN),
					ID(MODEY), ID(MODEZ), ID(CLK),	ID(CEX1),  ID(CEX2),	 ID(CEX3),	ID(CEXB),     ID(CEY1),	   ID(CEY2),
					ID(CEZ),   ID(CEM),   ID(CEP),	ID(CEPRE), ID(CEMODEIN), ID(CEMODEY),	ID(CEMODEZ),  ID(RSTX),	   ID(RSTXB),
					ID(RSTY),  ID(RSTZ),  ID(RSTM), ID(RSTP),  ID(RSTPRE),	 ID(RSTMODEIN), ID(RSTMODEY), ID(RSTMODEZ)},
		       {ID(COUT), ID(CPO), ID(CXO), ID(CXBO), ID(P)}, false);
	ct->setup_type(ID(GTP_BUF), {ID(I)}, {ID(Z)}, false);
	ct->setup_type(ID(GTP_CFGCLK), {ID(CLKIN), ID(CE_N)}, {}, false);
	ct->setup_type(ID(GTP_CLKBUFCE), {ID(CE), ID(CLKIN)}, {ID(CLKOUT)}, false);
	ct->setup_type(ID(GTP_CLKBUFG), {ID(CLKIN)}, {ID(CLKOUT)}, false);
	ct->setup_type(ID(GTP_CLKBUFGCE), {ID(CLKIN), ID(CE)}, {ID(CLKOUT)}, false);
	ct->setup_type(ID(GTP_CLKBUFGMUX), {ID(CLKIN0), ID(CLKIN1), ID(SEL)}, {ID(CLKOUT)}, false);
	ct->setup_type(ID(GTP_CLKBUFGMUX_E1), {ID(CLKIN0), ID(CLKIN1), ID(SEL), ID(EN)}, {ID(CLKOUT)}, false);
	ct->setup_type(ID(GTP_CLKBUFGMUX_E2), {ID(CLKIN0), ID(CLKIN1), ID(DETECT_CLK0), ID(DETECT_CLK1), ID(SEL)}, {ID(CLKOUT)}, false);
	ct->setup_type(ID(GTP_CLKBUFM), {ID(CLKIN)}, {ID(CLKOUT)}, false);
	ct->setup_type(ID(GTP_CLKBUFMCE), {ID(CLKIN), ID(CE)}, {ID(CLKOUT)}, false);
	ct->setup_type(ID(GTP_CLKBUFR), {ID(CLKIN)}, {ID(CLKOUT)}, false);
	ct->setup_type(ID(GTP_CLKBUFX), {ID(CLKIN)}, {ID(CLKOUT)}, false);
	ct->setup_type(ID(GTP_CLKBUFXCE), {ID(CLKIN), ID(CE)}, {ID(CLKOUT)}, false);
	ct->setup_type(ID(GTP_CLKPD), {ID(RST), ID(CLK_SAMPLE), ID(CLK_CTRL), ID(CLK_PHY), ID(DONE)}, {ID(FLAG_PD), ID(LOCK)}, false);
	ct->setup_type(ID(GTP_DDC_E2),
		       {ID(RST), ID(RST_TRAINING_N), ID(CLKA), ID(CLKB), ID(DQSI), ID(DQSIB), ID(DELAY_STEP0), ID(DELAY_STEP1), ID(DELAY_STEP2),
			ID(DELAY_STEP3), ID(DELAY_STEP4), ID(DQS_GATE_CTRL), ID(GATE_SEL), ID(CLK_GATE_CTRL), ID(CLKA_GATE)},
		       {ID(WCLK), ID(WCLK_DELAY), ID(DQSI_DELAY), ID(DQSIB_DELAY), ID(DGTS), ID(IFIFO_WADDR), ID(IFIFO_RADDR), ID(READ_VALID),
			ID(DQS_DRIFT), ID(DRIFT_DETECT_ERR), ID(DQS_DRIFT_STATUS), ID(DQS_SAMPLE)},
		       false);
	ct->setup_type(ID(GTP_DDC_E2_DFT),
		       {ID(RST), ID(RST_TRAINING_N), ID(CLKA), ID(CLKB), ID(DQSI), ID(DQSIB), ID(DELAY_STEP0), ID(DELAY_STEP1), ID(DELAY_STEP2),
			ID(DELAY_STEP3), ID(DELAY_STEP4), ID(DQS_GATE_CTRL), ID(GATE_SEL), ID(CLK_GATE_CTRL), ID(CLKA_GATE)},
		       {ID(WCLK), ID(WCLK_DELAY), ID(DQSI_DELAY), ID(DQSIB_DELAY), ID(DGTS), ID(IFIFO_WADDR), ID(IFIFO_RADDR), ID(READ_VALID),
			ID(DQS_DRIFT), ID(DRIFT_DETECT_ERR), ID(DQS_DRIFT_STATUS), ID(DQS_SAMPLE), ID(GATE_HIGHB), ID(GATE_HIGH_LATCHB)},
		       false);
	ct->setup_type(ID(GTP_DDC_E3),
		       {ID(RST), ID(RST_TRAINING_N), ID(CLKA), ID(CLKB), ID(DQSI), ID(DQSIB), ID(DELAY_STEP0), ID(DELAY_STEP1), ID(DELAY_STEP2),
			ID(DELAY_STEP3), ID(DELAY_STEP4), ID(DQS_GATE_CTRL), ID(GATE_SEL), ID(CLK_GATE_CTRL), ID(CLKA_GATE)},
		       {ID(WCLK), ID(WCLK_DELAY), ID(DQSI_DELAY), ID(DQSIB_DELAY), ID(DGTS), ID(IFIFO_WADDR), ID(IFIFO_RADDR), ID(READ_VALID),
			ID(DQS_DRIFT), ID(DRIFT_DETECT_ERR), ID(DQS_DRIFT_STATUS), ID(DQS_SAMPLE)},
		       false);
	ct->setup_type(ID(GTP_DFF), {ID(D), ID(CLK)}, {ID(Q)}, false);
	ct->setup_type(ID(GTP_DFF_C), {ID(D), ID(CLK), ID(C)}, {ID(Q)}, false);
	ct->setup_type(ID(GTP_DFF_CE), {ID(D), ID(CLK), ID(C), ID(CE)}, {ID(Q)}, false);
	ct->setup_type(ID(GTP_DFF_E), {ID(D), ID(CLK), ID(CE)}, {ID(Q)}, false);
	ct->setup_type(ID(GTP_DFF_P), {ID(D), ID(CLK), ID(P)}, {ID(Q)}, false);
	ct->setup_type(ID(GTP_DFF_PE), {ID(D), ID(CLK), ID(P), ID(CE)}, {ID(Q)}, false);
	ct->setup_type(ID(GTP_DFF_R), {ID(D), ID(CLK), ID(R)}, {ID(Q)}, false);
	ct->setup_type(ID(GTP_DFF_RE), {ID(D), ID(CLK), ID(R), ID(CE)}, {ID(Q)}, false);
	ct->setup_type(ID(GTP_DFF_S), {ID(D), ID(CLK), ID(S)}, {ID(Q)}, false);
	ct->setup_type(ID(GTP_DFF_SE), {ID(D), ID(CLK), ID(S), ID(CE)}, {ID(Q)}, false);
	ct->setup_type(ID(GTP_DLATCH), {ID(D), ID(G)}, {ID(Q)}, false);
	ct->setup_type(ID(GTP_DLATCH_C), {ID(D), ID(G), ID(C)}, {ID(Q)}, false);
	ct->setup_type(ID(GTP_DLATCH_CE), {ID(D), ID(G), ID(C), ID(GE)}, {ID(Q)}, false);
	ct->setup_type(ID(GTP_DLATCH_E), {ID(D), ID(G), ID(GE)}, {ID(Q)}, false);
	ct->setup_type(ID(GTP_DLATCH_P), {ID(D), ID(G), ID(P)}, {ID(Q)}, false);
	ct->setup_type(ID(GTP_DLATCH_PE), {ID(D), ID(G), ID(P), ID(GE)}, {ID(Q)}, false);
	ct->setup_type(ID(GTP_DLL_E2), {ID(CLKIN), ID(SYS_CLK), ID(PWD), ID(RST), ID(UPDATE_N)}, {ID(DELAY_STEP), ID(DELAY_STEP1), ID(LOCK)}, false);
	ct->setup_type(ID(GTP_DRM18K_E1),
		       {ID(DIA), ID(DIB), ID(ADDRA), ID(ADDRA_HOLD), ID(ADDRB), ID(ADDRB_HOLD), ID(BWEA), ID(BWEB), ID(CLKA), ID(CLKB), ID(CEA),
			ID(CEB), ID(WEA), ID(WEB), ID(ORCEA), ID(ORCEB), ID(RSTA), ID(RSTB)},
		       {ID(DOA), ID(DOB)}, false);
	ct->setup_type(ID(GTP_DRM36K_E1),
		       {ID(CINA),  ID(CINB),  ID(DIA),	ID(DIB),  ID(ADDRA),	      ID(ADDRA_HOLD),	 ID(ADDRB), ID(ADDRB_HOLD), ID(CSA),
			ID(CSB),   ID(BWEA),  ID(BWEB), ID(CLKA), ID(CLKB),	      ID(CEA),		 ID(CEB),   ID(WEA),	    ID(WEB),
			ID(ORCEA), ID(ORCEB), ID(RSTA), ID(RSTB), ID(INJECT_SBITERR), ID(INJECT_DBITERR)},
		       {ID(COUTA), ID(COUTB), ID(DOA), ID(DOB), ID(ECC_SBITERR), ID(ECC_DBITERR), ID(ECC_RDADDR), ID(ECC_PARITY)}, false);
	ct->setup_type(ID(GTP_EFUSECODE), {}, {ID(EFUSE_CODE)}, false);
	ct->setup_type(ID(GTP_FIFO18K_E1), {ID(DI), ID(WCLK), ID(RCLK), ID(WCE), ID(RCE), ID(ORCE), ID(RST)},
		       {ID(ALMOST_EMPTY), ID(ALMOST_FULL), ID(EMPTY), ID(FULL), ID(DO)}, false);
	ct->setup_type(ID(GTP_FIFO36K_E1), {ID(DI), ID(WCLK), ID(RCLK), ID(WCE), ID(RCE), ID(ORCE), ID(RST), ID(INJECT_SBITERR), ID(INJECT_DBITERR)},
		       {ID(ALMOST_EMPTY), ID(ALMOST_FULL), ID(EMPTY), ID(FULL), ID(DO), ID(ECC_SBITERR), ID(ECC_DBITERR)}, false);
	ct->setup_type(ID(GTP_GPLL), {ID(CLKIN1),      ID(CLKIN2),	ID(CLKFB),	 ID(CLKIN_SEL),	  ID(DPS_CLK),	   ID(DPS_EN),
				      ID(DPS_DIR),     ID(CLKOUT0_SYN), ID(CLKOUT1_SYN), ID(CLKOUT2_SYN), ID(CLKOUT3_SYN), ID(CLKOUT4_SYN),
				      ID(CLKOUT5_SYN), ID(CLKOUT6_SYN), ID(CLKOUTF_SYN), ID(PLL_PWD),	  ID(RST),	   ID(APB_CLK),
				      ID(APB_RST_N),   ID(APB_ADDR),	ID(APB_SEL),	 ID(APB_EN),	  ID(APB_WRITE),   ID(APB_WDATA)},
		       {ID(CLKOUT0), ID(CLKOUT0N), ID(CLKOUT1), ID(CLKOUT1N), ID(CLKOUT2), ID(CLKOUT2N), ID(CLKOUT3), ID(CLKOUT3N), ID(CLKOUT4),
			ID(CLKOUT5), ID(CLKOUT6), ID(CLKOUTF), ID(CLKOUTFN), ID(LOCK), ID(DPS_DONE), ID(APB_RDATA), ID(APB_READY)},
		       false);
	ct->setup_type(ID(GTP_GRS), {ID(GRS_N)}, {}, false);
	ct->setup_type(ID(GTP_HPIO_VREF), {ID(CODE_VREF0), ID(CODE_VREF1), ID(CODE_VREF2), ID(CODE_VREF3)}, {}, false);
	ct->setup_type(ID(GTP_HSSTHP_BUFDS), {ID(COM_POWERDOWN), ID(PAD_REFCLKP), ID(PAD_REFCLKN)}, {ID(REFCLK_OUTP), ID(PMA_REFCLK_TO_FABRIC)},
		       false);
	ct->setup_type(ID(GTP_HSSTHP_HPLL),
		       {ID(P_CFG_RST_HPLL),
			ID(P_CFG_CLK_HPLL),
			ID(P_CFG_PSEL_HPLL),
			ID(P_CFG_ENABLE_HPLL),
			ID(P_CFG_WRITE_HPLL),
			ID(P_CFG_ADDR_HPLL),
			ID(P_CFG_WDATA_HPLL),
			ID(P_HPLL_POWERDOWN),
			ID(P_HPLL_RST),
			ID(P_HPLL_LOCKDET_RST),
			ID(P_RES_CAL_RST),
			ID(P_TX_SYNC),
			ID(P_HPLL_DIV_SYNC),
			ID(P_REFCLK_DIV_SYNC),
			ID(P_HPLL_VCO_CALIB_EN),
			ID(P_RESCAL_I_CODE_I),
			ID(P_HPLL_DIV_CHANGE),
			ID(P_HPLL_REFCLK_I),
			ID(REFCLK_TO_TX_SYNC_I),
			ID(REFCLK_TO_REFCLK_SYNC_I),
			ID(REFCLK_TO_DIV_SYNC_I),
			ID(ANA_TX_SYNC_I),
			ID(ANA_HPLL_REFCLK_SYNC_I),
			ID(ANA_HPLL_DIV_SYNC_I),
			ID(P_PLL_REFCLK6_I),
			ID(REFCLK0),
			ID(REFCLK1),
			ID(REFCLK0_FROM_UPPER_HSST),
			ID(REFCLK1_FROM_UPPER_HSST),
			ID(REFCLK0_FROM_LOWER_HSST),
			ID(REFCLK1_FROM_LOWER_HSST),
			ID(REFCLK_FROM_FABRIC),
			ID(TX_SYNC_FROM_UPPER_HSST),
			ID(TX_SYNC_FROM_LOWER_HSST),
			ID(REFCLK_SYNC_FROM_UPPER_HSST),
			ID(REFCLK_SYNC_FROM_LOWER_HSST),
			ID(DIV_SYNC_FROM_UPPER_HSST),
			ID(DIV_SYNC_FROM_LOWER_HSST)},
		       {ID(P_CFG_READY_HPLL),
			ID(P_CFG_RDATA_HPLL),
			ID(P_CFG_INT_HPLL),
			ID(P_RES_CAL_CODE_FABRIC),
			ID(P_HPLL_READY),
			ID(PMA_HPLL_READY_O),
			ID(PMA_HPLL_REFCLK_O),
			ID(PMA_TX_SYNC_HPLL_O),
			ID(PMA_HPLL_CK0),
			ID(PMA_HPLL_CK90),
			ID(PMA_HPLL_CK180),
			ID(PMA_HPLL_CK270),
			ID(TX_SYNC_REFSYNC_O),
			ID(REFCLK_SYNC_REFSYNC_O),
			ID(DIV_SYNC_REFSYNC_O),
			ID(LPLL_REFCKOUT_CH0),
			ID(LPLL_REFCKOUT_CH1),
			ID(LPLL_REFCKOUT_CH2),
			ID(LPLL_REFCKOUT_CH3),
			ID(REFCLK0_FOR_UPPER_HSST),
			ID(REFCLK1_FOR_UPPER_HSST),
			ID(REFCLK0_FOR_LOWER_HSST),
			ID(REFCLK1_FOR_LOWER_HSST),
			ID(TX_SYNC_FOR_UPPER_HSST),
			ID(TX_SYNC_FOR_LOWER_HSST),
			ID(ANA_TX_SYNC_O),
			ID(REFCLK_SYNC_FOR_UPPER_HSST),
			ID(REFCLK_SYNC_FOR_LOWER_HSST),
			ID(DIV_SYNC_FOR_UPPER_HSST),
			ID(DIV_SYNC_FOR_LOWER_HSST)},
		       false);
	ct->setup_type(ID(GTP_HSSTHP_HPLL_DFT),
		       {ID(P_CFG_RST_HPLL),
			ID(P_CFG_CLK_HPLL),
			ID(P_CFG_PSEL_HPLL),
			ID(P_CFG_ENABLE_HPLL),
			ID(P_CFG_WRITE_HPLL),
			ID(P_CFG_ADDR_HPLL),
			ID(P_CFG_WDATA_HPLL),
			ID(P_HPLL_POWERDOWN),
			ID(P_HPLL_RST),
			ID(P_HPLL_LOCKDET_RST),
			ID(P_RES_CAL_RST),
			ID(P_TX_SYNC),
			ID(P_HPLL_DIV_SYNC),
			ID(P_REFCLK_DIV_SYNC),
			ID(P_HPLL_VCO_CALIB_EN),
			ID(P_RESCAL_I_CODE_I),
			ID(P_HPLL_DIV_CHANGE),
			ID(P_HPLL_REFCLK_I),
			ID(REFCLK_TO_TX_SYNC_I),
			ID(REFCLK_TO_REFCLK_SYNC_I),
			ID(REFCLK_TO_DIV_SYNC_I),
			ID(ANA_TX_SYNC_I),
			ID(ANA_HPLL_REFCLK_SYNC_I),
			ID(ANA_HPLL_DIV_SYNC_I),
			ID(P_PLL_REFCLK6_I),
			ID(REFCLK0),
			ID(REFCLK1),
			ID(REFCLK0_FROM_UPPER_HSST),
			ID(REFCLK1_FROM_UPPER_HSST),
			ID(REFCLK0_FROM_LOWER_HSST),
			ID(REFCLK1_FROM_LOWER_HSST),
			ID(REFCLK_FROM_FABRIC),
			ID(TX_SYNC_FROM_UPPER_HSST),
			ID(TX_SYNC_FROM_LOWER_HSST),
			ID(REFCLK_SYNC_FROM_UPPER_HSST),
			ID(REFCLK_SYNC_FROM_LOWER_HSST),
			ID(DIV_SYNC_FROM_UPPER_HSST),
			ID(DIV_SYNC_FROM_LOWER_HSST),
			ID(P_TEST_SE_N),
			ID(P_TEST_MODE_N),
			ID(P_TEST_RSTN),
			ID(P_TEST_SI0),
			ID(P_TEST_SI1),
			ID(P_FOR_PMA_TEST_MODE_N),
			ID(P_FOR_PMA_TEST_SE_N),
			ID(P_FOR_PMA_TEST_CLK),
			ID(P_FOR_PMA_TEST_RSTN),
			ID(P_FOR_PMA_TEST_SI)},
		       {ID(P_CFG_READY_HPLL),
			ID(P_CFG_RDATA_HPLL),
			ID(P_CFG_INT_HPLL),
			ID(P_RES_CAL_CODE_FABRIC),
			ID(P_HPLL_READY),
			ID(PMA_HPLL_READY_O),
			ID(PMA_HPLL_REFCLK_O),
			ID(PMA_TX_SYNC_HPLL_O),
			ID(PMA_HPLL_CK0),
			ID(PMA_HPLL_CK90),
			ID(PMA_HPLL_CK180),
			ID(PMA_HPLL_CK270),
			ID(TX_SYNC_REFSYNC_O),
			ID(REFCLK_SYNC_REFSYNC_O),
			ID(DIV_SYNC_REFSYNC_O),
			ID(LPLL_REFCKOUT_CH0),
			ID(LPLL_REFCKOUT_CH1),
			ID(LPLL_REFCKOUT_CH2),
			ID(LPLL_REFCKOUT_CH3),
			ID(REFCLK0_FOR_UPPER_HSST),
			ID(REFCLK1_FOR_UPPER_HSST),
			ID(REFCLK0_FOR_LOWER_HSST),
			ID(REFCLK1_FOR_LOWER_HSST),
			ID(TX_SYNC_FOR_UPPER_HSST),
			ID(TX_SYNC_FOR_LOWER_HSST),
			ID(ANA_TX_SYNC_O),
			ID(REFCLK_SYNC_FOR_UPPER_HSST),
			ID(REFCLK_SYNC_FOR_LOWER_HSST),
			ID(DIV_SYNC_FOR_UPPER_HSST),
			ID(DIV_SYNC_FOR_LOWER_HSST),
			ID(P_TEST_SO0),
			ID(P_TEST_SO1),
			ID(P_FOR_PMA_TEST_SO)},
		       false);
	ct->setup_type(ID(GTP_HSSTHP_HPLL_t),
		       {ID(P_CFG_RST_HPLL),
			ID(P_CFG_CLK_HPLL),
			ID(P_CFG_PSEL_HPLL),
			ID(P_CFG_ENABLE_HPLL),
			ID(P_CFG_WRITE_HPLL),
			ID(P_CFG_ADDR_HPLL),
			ID(P_CFG_WDATA_HPLL),
			ID(P_COM_POWERDOWN),
			ID(P_HPLL_POWERDOWN),
			ID(P_HPLL_RST),
			ID(P_HPLL_LOCKDET_RST),
			ID(P_RES_CAL_RST),
			ID(P_TX_SYNC),
			ID(P_TX_RATE_CHANGE_ON_0),
			ID(P_TX_RATE_CHANGE_ON_1),
			ID(P_HPLL_DIV_SYNC),
			ID(P_REFCLK_DIV_SYNC),
			ID(P_HPLL_VCO_CALIB_EN),
			ID(P_RESCAL_I_CODE_I),
			ID(P_HPLL_REF_CLK),
			ID(P_HPLL_DIV_CHANGE),
			ID(P_FROM_LOWER_HSST_BUS),
			ID(P_FROM_UPPER_HSST_BUS),
			ID(PAD_REFCLKN_0),
			ID(PAD_REFCLKP_0),
			ID(PAD_REFCLKN_1),
			ID(PAD_REFCLKP_1)},
		       {ID(P_CFG_READY_HPLL),
			ID(P_CFG_RDATA_HPLL),
			ID(P_CFG_INT_HPLL),
			ID(P_RES_CAL_CODE_FABRIC),
			ID(P_REFCK2CORE_0),
			ID(P_REFCK2CORE_1),
			ID(P_HPLL_READY),
			ID(PMA_HPLL_READY_LEFT),
			ID(PMA_HPLL_READY_RIGHT),
			ID(PMA_HPLL_REFCLK_LEFT),
			ID(PMA_HPLL_REFCLK_RIGHT),
			ID(PMA_LPLL_REFCKOUT_CH0),
			ID(PMA_LPLL_REFCKOUT_CH1),
			ID(PMA_LPLL_REFCKOUT_CH2),
			ID(PMA_LPLL_REFCKOUT_CH3),
			ID(PMA_RES_CAL_LEFT),
			ID(PMA_RES_CAL_RIGHT),
			ID(PMA_TX_RATE_CHANGE_ON0_LEFT),
			ID(PMA_TX_RATE_CHANGE_ON0_RIGHT),
			ID(PMA_TX_RATE_CHANGE_ON1_LEFT),
			ID(PMA_TX_RATE_CHANGE_ON1_RIGHT),
			ID(PMA_TX_SYNC_HPLL_LEFT),
			ID(PMA_TX_SYNC_HPLL_RIGHT),
			ID(PMA_TX_SYNC_LEFT),
			ID(PMA_TX_SYNC_RIGHT),
			ID(PMA_HPLL_CK0_CH0),
			ID(PMA_HPLL_CK0_CH1),
			ID(PMA_HPLL_CK0_CH2),
			ID(PMA_HPLL_CK0_CH3),
			ID(PMA_HPLL_CK90_CH0),
			ID(PMA_HPLL_CK90_CH1),
			ID(PMA_HPLL_CK90_CH2),
			ID(PMA_HPLL_CK90_CH3),
			ID(PMA_HPLL_CK180_CH0),
			ID(PMA_HPLL_CK180_CH1),
			ID(PMA_HPLL_CK180_CH2),
			ID(PMA_HPLL_CK180_CH3),
			ID(PMA_HPLL_CK270_CH0),
			ID(PMA_HPLL_CK270_CH1),
			ID(PMA_HPLL_CK270_CH2),
			ID(PMA_HPLL_CK270_CH3),
			ID(P_FOR_LOWER_HSST_BUS),
			ID(P_FOR_UPPER_HSST_BUS)},
		       false);
	ct->setup_type(ID(GTP_HSSTHP_LANE),
		       {ID(P_RX_CLK_FR_CORE),
			ID(P_RCLK2_FR_CORE),
			ID(P_TX_CLK_FR_CORE),
			ID(P_TCLK2_FR_CORE),
			ID(P_PCS_RX_RST),
			ID(P_PCS_TX_RST),
			ID(P_EXT_BRIDGE_PCS_RST),
			ID(P_CFG_RST),
			ID(P_CFG_CLK),
			ID(P_CFG_PSEL),
			ID(P_CFG_ENABLE),
			ID(P_CFG_WRITE),
			ID(P_CFG_ADDR),
			ID(P_CFG_WDATA),
			ID(LANE_CIN_BUS_FORWARD),
			ID(LANE_CIN_BUS_BACKWARD),
			ID(P_TDATA),
			ID(P_PCIE_EI_H),
			ID(P_PCIE_EI_L),
			ID(P_TX_DEEMP),
			ID(P_TX_DEEMP_POST_SEL),
			ID(P_BLK_ALIGN_CTRL),
			ID(P_TX_ENC_TYPE),
			ID(P_RX_DEC_TYPE),
			ID(P_PCS_BIT_SLIP),
			ID(P_PCS_WORD_ALIGN_EN),
			ID(P_RX_POLARITY_INVERT),
			ID(P_PCS_MCB_EXT_EN),
			ID(P_PCS_NEAREND_LOOP),
			ID(P_PCS_FAREND_LOOP),
			ID(P_PMA_NEAREND_PLOOP),
			ID(P_PMA_NEAREND_SLOOP),
			ID(P_PMA_FAREND_PLOOP),
			ID(P_PCS_PRBS_EN),
			ID(P_LANE_POWERDOWN),
			ID(P_LANE_RST),
			ID(P_RX_LANE_POWERDOWN),
			ID(P_RX_PMA_RST),
			ID(P_RX_CDR_RST),
			ID(P_RX_CLKPATH_RST),
			ID(P_RX_DFE_RST),
			ID(P_RX_LEQ_RST),
			ID(P_RX_SLIDING_RST),
			ID(P_RX_DFE_EN),
			ID(P_RX_T1_EN),
			ID(P_RX_CDRX_EN),
			ID(P_RX_T1_DFE_EN),
			ID(P_RX_T2_DFE_EN),
			ID(P_RX_T3_DFE_EN),
			ID(P_RX_T4_DFE_EN),
			ID(P_RX_T5_DFE_EN),
			ID(P_RX_T6_DFE_EN),
			ID(P_RX_SLIDING_EN),
			ID(P_RX_EYE_RST),
			ID(P_RX_EYE_EN),
			ID(P_RX_EYE_TAP),
			ID(P_RX_PIC_EYE),
			ID(P_RX_PIC_FASTLOCK),
			ID(P_RX_PIC_FASTLOCK_STROBE),
			ID(P_EM_RD_TRIGGER),
			ID(P_EM_MODE_CTRL),
			ID(P_RX_CTLE_DCCAL_RST),
			ID(P_RX_SLICER_DCCAL_RST),
			ID(P_RX_SLICER_DCCAL_EN),
			ID(P_RX_CTLE_DCCAL_EN),
			ID(P_RX_SLIP_RST),
			ID(P_RX_SLIP_EN),
			ID(P_LPLL_POWERDOWN),
			ID(P_LPLL_RST),
			ID(P_LPLL_LOCKDET_RST),
			ID(P_TX_LS_DATA),
			ID(P_TX_BEACON_EN),
			ID(P_TX_SWING),
			ID(P_TX_RXDET_REQ),
			ID(P_TX_RATE),
			ID(P_TX_BUSWIDTH),
			ID(P_TX_FREERUN_BUSWIDTH),
			ID(P_TX_MARGIN),
			ID(P_TX_PMA_RST),
			ID(P_TX_LANE_POWERDOWN),
			ID(P_TX_PIC_EN),
			ID(P_RX_RATE),
			ID(P_RX_BUSWIDTH),
			ID(P_RX_HIGHZ),
			ID(P_CIM_CLK_ALIGNER_RX),
			ID(P_CIM_CLK_ALIGNER_TX),
			ID(P_ALIGN_MODE_VALID_RX),
			ID(P_ALIGN_MODE_RX),
			ID(P_ALIGN_MODE_VALID_TX),
			ID(P_ALIGN_MODE_TX),
			ID(PMA_HPLL_CK0),
			ID(PMA_HPLL_CK90),
			ID(PMA_HPLL_CK180),
			ID(PMA_HPLL_CK270),
			ID(PMA_HPLL_READY_IN),
			ID(PMA_HPLL_REFCLK_IN),
			ID(PMA_TX_SYNC_HPLL_IN),
			ID(P_LPLL_REFCLK_IN),
			ID(P_TX_RATE_CHANGE_ON_0),
			ID(P_TX_RATE_CHANGE_ON_1),
			ID(P_TX_SYNC),
			ID(P_RX_SDN),
			ID(P_RX_SDP)},
		       {ID(P_CFG_READY),
			ID(P_CFG_RDATA),
			ID(P_CFG_INT),
			ID(LANE_COUT_BUS_FORWARD),
			ID(LANE_COUT_BUS_BACKWARD),
			ID(P_RX_PRBS_ERROR),
			ID(P_PCS_RX_MCB_STATUS),
			ID(P_PCS_LSM_SYNCED),
			ID(P_RDATA),
			ID(P_RXDVLD),
			ID(P_RXDVLD_H),
			ID(P_RXSTATUS),
			ID(P_EM_ERROR_CNT),
			ID(P_LPLL_READY),
			ID(P_RX_SIGDET_STATUS),
			ID(P_RX_SATA_COMINIT),
			ID(P_RX_SATA_COMWAKE),
			ID(P_RX_LS_DATA),
			ID(P_RX_READY),
			ID(P_TEST_STATUS),
			ID(P_TX_RXDET_STATUS),
			ID(P_RCLK2FABRIC),
			ID(P_TCLK2FABRIC),
			ID(P_CA_ALIGN_RX),
			ID(P_CA_ALIGN_TX),
			ID(P_TX_SDN),
			ID(P_TX_SDP)},
		       false);
	ct->setup_type(ID(GTP_HSSTHP_LANE_DFT),
		       {ID(P_RX_CLK_FR_CORE),
			ID(P_RCLK2_FR_CORE),
			ID(P_TX_CLK_FR_CORE),
			ID(P_TCLK2_FR_CORE),
			ID(P_PCS_RX_RST),
			ID(P_PCS_TX_RST),
			ID(P_EXT_BRIDGE_PCS_RST),
			ID(P_CFG_RST),
			ID(P_CFG_CLK),
			ID(P_CFG_PSEL),
			ID(P_CFG_ENABLE),
			ID(P_CFG_WRITE),
			ID(P_CFG_ADDR),
			ID(P_CFG_WDATA),
			ID(LANE_CIN_BUS_FORWARD),
			ID(LANE_CIN_BUS_BACKWARD),
			ID(P_TDATA),
			ID(P_PCIE_EI_H),
			ID(P_PCIE_EI_L),
			ID(P_TX_DEEMP),
			ID(P_TX_DEEMP_POST_SEL),
			ID(P_BLK_ALIGN_CTRL),
			ID(P_TX_ENC_TYPE),
			ID(P_RX_DEC_TYPE),
			ID(P_PCS_BIT_SLIP),
			ID(P_PCS_WORD_ALIGN_EN),
			ID(P_RX_POLARITY_INVERT),
			ID(P_PCS_MCB_EXT_EN),
			ID(P_PCS_NEAREND_LOOP),
			ID(P_PCS_FAREND_LOOP),
			ID(P_PMA_NEAREND_PLOOP),
			ID(P_PMA_NEAREND_SLOOP),
			ID(P_PMA_FAREND_PLOOP),
			ID(P_PCS_PRBS_EN),
			ID(P_LANE_POWERDOWN),
			ID(P_LANE_RST),
			ID(P_RX_LANE_POWERDOWN),
			ID(P_RX_PMA_RST),
			ID(P_RX_CDR_RST),
			ID(P_RX_CLKPATH_RST),
			ID(P_RX_DFE_RST),
			ID(P_RX_LEQ_RST),
			ID(P_RX_SLIDING_RST),
			ID(P_RX_DFE_EN),
			ID(P_RX_T1_EN),
			ID(P_RX_CDRX_EN),
			ID(P_RX_T1_DFE_EN),
			ID(P_RX_T2_DFE_EN),
			ID(P_RX_T3_DFE_EN),
			ID(P_RX_T4_DFE_EN),
			ID(P_RX_T5_DFE_EN),
			ID(P_RX_T6_DFE_EN),
			ID(P_RX_SLIDING_EN),
			ID(P_RX_EYE_RST),
			ID(P_RX_EYE_EN),
			ID(P_RX_EYE_TAP),
			ID(P_RX_PIC_EYE),
			ID(P_RX_PIC_FASTLOCK),
			ID(P_RX_PIC_FASTLOCK_STROBE),
			ID(P_EM_RD_TRIGGER),
			ID(P_EM_MODE_CTRL),
			ID(P_RX_CTLE_DCCAL_RST),
			ID(P_RX_SLICER_DCCAL_RST),
			ID(P_RX_SLICER_DCCAL_EN),
			ID(P_RX_CTLE_DCCAL_EN),
			ID(P_RX_SLIP_RST),
			ID(P_RX_SLIP_EN),
			ID(P_LPLL_POWERDOWN),
			ID(P_LPLL_RST),
			ID(P_LPLL_LOCKDET_RST),
			ID(P_TX_LS_DATA),
			ID(P_TX_BEACON_EN),
			ID(P_TX_SWING),
			ID(P_TX_RXDET_REQ),
			ID(P_TX_RATE),
			ID(P_TX_BUSWIDTH),
			ID(P_TX_FREERUN_BUSWIDTH),
			ID(P_TX_MARGIN),
			ID(P_TX_PMA_RST),
			ID(P_TX_LANE_POWERDOWN),
			ID(P_TX_PIC_EN),
			ID(P_RX_RATE),
			ID(P_RX_BUSWIDTH),
			ID(P_RX_HIGHZ),
			ID(P_CIM_CLK_ALIGNER_RX),
			ID(P_CIM_CLK_ALIGNER_TX),
			ID(P_ALIGN_MODE_VALID_RX),
			ID(P_ALIGN_MODE_RX),
			ID(P_ALIGN_MODE_VALID_TX),
			ID(P_ALIGN_MODE_TX),
			ID(PMA_HPLL_CK0),
			ID(PMA_HPLL_CK90),
			ID(PMA_HPLL_CK180),
			ID(PMA_HPLL_CK270),
			ID(PMA_HPLL_READY_IN),
			ID(PMA_HPLL_REFCLK_IN),
			ID(PMA_TX_SYNC_HPLL_IN),
			ID(P_LPLL_REFCLK_IN),
			ID(P_TX_RATE_CHANGE_ON_0),
			ID(P_TX_RATE_CHANGE_ON_1),
			ID(P_TX_SYNC),
			ID(P_RX_SDN),
			ID(P_RX_SDP),
			ID(P_TEST_SE_N),
			ID(P_TEST_MODE_N),
			ID(P_TEST_RSTN),
			ID(P_TEST_SI0),
			ID(P_TEST_SI1),
			ID(P_TEST_SI2),
			ID(P_TEST_SI3),
			ID(P_TEST_SI4),
			ID(P_FOR_PMA_TEST_MODE_N),
			ID(P_FOR_PMA_TEST_SE_N),
			ID(P_FOR_PMA_TEST_CLK),
			ID(P_FOR_PMA_TEST_RSTN),
			ID(P_FOR_PMA_TEST_SI)},
		       {ID(P_CFG_READY),
			ID(P_CFG_RDATA),
			ID(P_CFG_INT),
			ID(LANE_COUT_BUS_FORWARD),
			ID(LANE_COUT_BUS_BACKWARD),
			ID(P_RX_PRBS_ERROR),
			ID(P_PCS_RX_MCB_STATUS),
			ID(P_PCS_LSM_SYNCED),
			ID(P_RDATA),
			ID(P_RXDVLD),
			ID(P_RXDVLD_H),
			ID(P_RXSTATUS),
			ID(P_EM_ERROR_CNT),
			ID(P_LPLL_READY),
			ID(P_RX_SIGDET_STATUS),
			ID(P_RX_SATA_COMINIT),
			ID(P_RX_SATA_COMWAKE),
			ID(P_RX_LS_DATA),
			ID(P_RX_READY),
			ID(P_TEST_STATUS),
			ID(P_TX_RXDET_STATUS),
			ID(P_RCLK2FABRIC),
			ID(P_TCLK2FABRIC),
			ID(P_CA_ALIGN_RX),
			ID(P_CA_ALIGN_TX),
			ID(P_TX_SDN),
			ID(P_TX_SDP),
			ID(P_TEST_SO0),
			ID(P_TEST_SO1),
			ID(P_TEST_SO2),
			ID(P_TEST_SO3),
			ID(P_TEST_SO4),
			ID(P_FOR_PMA_TEST_SO)},
		       false);
	ct->setup_type(ID(GTP_HSSTHP_LANE_E1),
		       {ID(P_RX_CLK_FR_CORE),
			ID(P_RCLK2_FR_CORE),
			ID(P_TX_CLK_FR_CORE),
			ID(P_TCLK2_FR_CORE),
			ID(P_PCS_RX_RST),
			ID(P_PCS_TX_RST),
			ID(P_EXT_BRIDGE_PCS_RST),
			ID(P_CFG_RST),
			ID(P_CFG_CLK),
			ID(P_CFG_PSEL),
			ID(P_CFG_ENABLE),
			ID(P_CFG_WRITE),
			ID(P_CFG_ADDR),
			ID(P_CFG_WDATA),
			ID(LANE_CIN_BUS_FORWARD),
			ID(LANE_CIN_BUS_BACKWARD),
			ID(P_TDATA),
			ID(P_PCIE_EI_H),
			ID(P_PCIE_EI_L),
			ID(P_TX_DEEMP),
			ID(P_TX_DEEMP_POST_SEL),
			ID(P_BLK_ALIGN_CTRL),
			ID(P_TX_ENC_TYPE),
			ID(P_RX_DEC_TYPE),
			ID(P_PCS_BIT_SLIP),
			ID(P_PCS_WORD_ALIGN_EN),
			ID(P_RX_POLARITY_INVERT),
			ID(P_PCS_MCB_EXT_EN),
			ID(P_PCS_NEAREND_LOOP),
			ID(P_PCS_FAREND_LOOP),
			ID(P_PMA_NEAREND_PLOOP),
			ID(P_PMA_NEAREND_SLOOP),
			ID(P_PMA_FAREND_PLOOP),
			ID(P_PCS_PRBS_EN),
			ID(P_LANE_POWERDOWN),
			ID(P_LANE_RST),
			ID(P_RX_LANE_POWERDOWN),
			ID(P_RX_PMA_RST),
			ID(P_RX_CDR_RST),
			ID(P_RX_CLKPATH_RST),
			ID(P_RX_DFE_RST),
			ID(P_RX_LEQ_RST),
			ID(P_RX_SLIDING_RST),
			ID(P_RX_DFE_EN),
			ID(P_RX_T1_EN),
			ID(P_RX_CDRX_EN),
			ID(P_RX_T1_DFE_EN),
			ID(P_RX_T2_DFE_EN),
			ID(P_RX_T3_DFE_EN),
			ID(P_RX_T4_DFE_EN),
			ID(P_RX_T5_DFE_EN),
			ID(P_RX_T6_DFE_EN),
			ID(P_RX_SLIDING_EN),
			ID(P_RX_EYE_RST),
			ID(P_RX_EYE_EN),
			ID(P_RX_EYE_TAP),
			ID(P_RX_PIC_EYE),
			ID(P_RX_PIC_FASTLOCK),
			ID(P_RX_PIC_FASTLOCK_STROBE),
			ID(P_EM_RD_TRIGGER),
			ID(P_EM_MODE_CTRL),
			ID(P_RX_CTLE_DCCAL_RST),
			ID(P_RX_SLICER_DCCAL_RST),
			ID(P_RX_SLICER_DCCAL_EN),
			ID(P_RX_CTLE_DCCAL_EN),
			ID(P_RX_SLIP_RST),
			ID(P_RX_SLIP_EN),
			ID(P_LPLL_POWERDOWN),
			ID(P_LPLL_RST),
			ID(P_LPLL_LOCKDET_RST),
			ID(P_TX_LS_DATA),
			ID(P_TX_BEACON_EN),
			ID(P_TX_SWING),
			ID(P_TX_RXDET_REQ),
			ID(P_TX_RATE),
			ID(P_TX_BUSWIDTH),
			ID(P_TX_FREERUN_BUSWIDTH),
			ID(P_TX_MARGIN),
			ID(P_TX_PMA_RST),
			ID(P_TX_LANE_POWERDOWN),
			ID(P_TX_PIC_EN),
			ID(P_RX_RATE),
			ID(P_RX_BUSWIDTH),
			ID(P_RX_HIGHZ),
			ID(P_CIM_CLK_ALIGNER_RX),
			ID(P_CIM_CLK_ALIGNER_TX),
			ID(P_ALIGN_MODE_VALID_RX),
			ID(P_ALIGN_MODE_RX),
			ID(P_ALIGN_MODE_VALID_TX),
			ID(P_ALIGN_MODE_TX),
			ID(PMA_HPLL_CK0),
			ID(PMA_HPLL_CK90),
			ID(PMA_HPLL_CK180),
			ID(PMA_HPLL_CK270),
			ID(PMA_HPLL_READY_IN),
			ID(PMA_HPLL_REFCLK_IN),
			ID(PMA_TX_SYNC_HPLL_IN),
			ID(P_LPLL_REFCLK_IN),
			ID(P_TX_RATE_CHANGE_ON_0),
			ID(P_TX_RATE_CHANGE_ON_1),
			ID(P_TX_SYNC),
			ID(P_RX_SDN),
			ID(P_RX_SDP),
			ID(P_CDR_PICTRL_OW),
			ID(P_CDR_PICTRL_OW_VAL)},
		       {ID(P_CFG_READY),
			ID(P_CFG_RDATA),
			ID(P_CFG_INT),
			ID(LANE_COUT_BUS_FORWARD),
			ID(LANE_COUT_BUS_BACKWARD),
			ID(P_RX_PRBS_ERROR),
			ID(P_PCS_RX_MCB_STATUS),
			ID(P_PCS_LSM_SYNCED),
			ID(P_RDATA),
			ID(P_RXDVLD),
			ID(P_RXDVLD_H),
			ID(P_RXSTATUS),
			ID(P_EM_ERROR_CNT),
			ID(P_LPLL_READY),
			ID(P_RX_SIGDET_STATUS),
			ID(P_RX_SATA_COMINIT),
			ID(P_RX_SATA_COMWAKE),
			ID(P_RX_LS_DATA),
			ID(P_RX_READY),
			ID(P_TEST_STATUS),
			ID(P_TX_RXDET_STATUS),
			ID(P_RCLK2FABRIC),
			ID(P_TCLK2FABRIC),
			ID(P_CA_ALIGN_RX),
			ID(P_CA_ALIGN_TX),
			ID(P_TX_SDN),
			ID(P_TX_SDP)},
		       false);
	ct->setup_type(ID(GTP_HSSTHP_LANE_t),
		       {ID(P_RX_CLK_FR_CORE),
			ID(P_RCLK2_FR_CORE),
			ID(P_TX_CLK_FR_CORE),
			ID(P_TCLK2_FR_CORE),
			ID(P_PCS_RX_RST),
			ID(P_PCS_TX_RST),
			ID(P_EXT_BRIDGE_PCS_RST),
			ID(P_CFG_RST),
			ID(P_CFG_CLK),
			ID(P_CFG_PSEL),
			ID(P_CFG_ENABLE),
			ID(P_CFG_WRITE),
			ID(P_CFG_ADDR),
			ID(P_CFG_WDATA),
			ID(LANE_CIN_BUS_FORWARD),
			ID(LANE_CIN_BUS_BACKWARD),
			ID(P_TDATA),
			ID(P_PCIE_EI_H),
			ID(P_PCIE_EI_L),
			ID(P_TX_DEEMP),
			ID(P_TX_DEEMP_POST_SEL),
			ID(P_BLK_ALIGN_CTRL),
			ID(P_TX_ENC_TYPE),
			ID(P_RX_DEC_TYPE),
			ID(P_PCS_BIT_SLIP),
			ID(P_PCS_WORD_ALIGN_EN),
			ID(P_RX_POLARITY_INVERT),
			ID(P_PCS_MCB_EXT_EN),
			ID(P_PCS_NEAREND_LOOP),
			ID(P_PCS_FAREND_LOOP),
			ID(P_PMA_NEAREND_PLOOP),
			ID(P_PMA_NEAREND_SLOOP),
			ID(P_PMA_FAREND_PLOOP),
			ID(P_PCS_PRBS_EN),
			ID(P_LANE_POWERDOWN),
			ID(P_LANE_RST),
			ID(P_RX_LANE_POWERDOWN),
			ID(P_RX_PMA_RST),
			ID(P_RX_CDR_RST),
			ID(P_RX_CLKPATH_RST),
			ID(P_RX_DFE_RST),
			ID(P_RX_LEQ_RST),
			ID(P_RX_SLIDING_RST),
			ID(P_RX_DFE_EN),
			ID(P_RX_T1_EN),
			ID(P_RX_CDRX_EN),
			ID(P_RX_T1_DFE_EN),
			ID(P_RX_T2_DFE_EN),
			ID(P_RX_T3_DFE_EN),
			ID(P_RX_T4_DFE_EN),
			ID(P_RX_T5_DFE_EN),
			ID(P_RX_T6_DFE_EN),
			ID(P_RX_SLIDING_EN),
			ID(P_RX_EYE_RST),
			ID(P_RX_EYE_EN),
			ID(P_RX_EYE_TAP),
			ID(P_RX_PIC_EYE),
			ID(P_RX_PIC_FASTLOCK),
			ID(P_RX_PIC_FASTLOCK_STROBE),
			ID(P_EM_RD_TRIGGER),
			ID(P_EM_MODE_CTRL),
			ID(P_RX_CTLE_DCCAL_RST),
			ID(P_RX_SLICER_DCCAL_RST),
			ID(P_RX_SLICER_DCCAL_EN),
			ID(P_RX_CTLE_DCCAL_EN),
			ID(P_RX_SLIP_RST),
			ID(P_RX_SLIP_EN),
			ID(P_LPLL_POWERDOWN),
			ID(P_LPLL_RST),
			ID(P_LPLL_LOCKDET_RST),
			ID(P_TX_LS_DATA),
			ID(P_TX_BEACON_EN),
			ID(P_TX_SWING),
			ID(P_TX_RXDET_REQ),
			ID(P_TX_RATE),
			ID(P_TX_BUSWIDTH),
			ID(P_TX_FREERUN_BUSWIDTH),
			ID(P_TX_MARGIN),
			ID(P_TX_PMA_RST),
			ID(P_TX_LANE_POWERDOWN),
			ID(P_TX_PIC_EN),
			ID(P_RX_RATE),
			ID(P_RX_BUSWIDTH),
			ID(P_RX_HIGHZ),
			ID(P_CIM_CLK_ALIGNER_RX),
			ID(P_CIM_CLK_ALIGNER_TX),
			ID(P_ALIGN_MODE_VALID_RX),
			ID(P_ALIGN_MODE_RX),
			ID(P_ALIGN_MODE_VALID_TX),
			ID(P_ALIGN_MODE_TX),
			ID(PMA_HPLL_CK0),
			ID(PMA_HPLL_CK90),
			ID(PMA_HPLL_CK180),
			ID(PMA_HPLL_CK270),
			ID(PMA_HPLL_READY_IN),
			ID(PMA_HPLL_REFCLK_IN),
			ID(PMA_IPN50U_IN),
			ID(PMA_LPLL_REFCLK),
			ID(PMA_RES_CAL_I),
			ID(PMA_TX_SYNC_HPLL),
			ID(PMA_TX_RATE_CHANGE_ON_0),
			ID(PMA_TX_RATE_CHANGE_ON_1),
			ID(PMA_TX_SYNC),
			ID(P_RX_SDN),
			ID(P_RX_SDP)},
		       {ID(P_CFG_READY),
			ID(P_CFG_RDATA),
			ID(P_CFG_INT),
			ID(LANE_COUT_BUS_FORWARD),
			ID(LANE_COUT_BUS_BACKWARD),
			ID(P_RX_PRBS_ERROR),
			ID(P_PCS_RX_MCB_STATUS),
			ID(P_PCS_LSM_SYNCED),
			ID(P_RDATA),
			ID(P_RXDVLD),
			ID(P_RXDVLD_H),
			ID(P_RXSTATUS),
			ID(P_EM_ERROR_CNT),
			ID(P_LPLL_READY),
			ID(P_RX_SIGDET_STATUS),
			ID(P_RX_SATA_COMINIT),
			ID(P_RX_SATA_COMWAKE),
			ID(P_RX_LS_DATA),
			ID(P_RX_READY),
			ID(P_TEST_STATUS),
			ID(P_TX_RXDET_STATUS),
			ID(P_RCLK2FABRIC),
			ID(P_TCLK2FABRIC),
			ID(P_CA_ALIGN_RX),
			ID(P_CA_ALIGN_TX),
			ID(P_TX_SDN),
			ID(P_TX_SDP)},
		       false);
	ct->setup_type(ID(GTP_IDDR_E1), {ID(D), ID(CLK), ID(CE), ID(RS)}, {ID(Q0), ID(Q1)}, false);
	ct->setup_type(ID(GTP_INBUF), {ID(I)}, {ID(O)}, false);
	ct->setup_type(ID(GTP_INBUFDS), {ID(I), ID(IB)}, {ID(O)}, false);
	ct->setup_type(ID(GTP_INBUFDS_E1), {ID(I), ID(IB)}, {ID(O), ID(OB)}, false);
	ct->setup_type(ID(GTP_INBUFE), {ID(EN), ID(I)}, {ID(O)}, false);
	ct->setup_type(ID(GTP_INBUFEDS), {ID(EN), ID(I), ID(IB)}, {ID(O)}, false);
	ct->setup_type(ID(GTP_INBUFEDS_E1), {ID(EN), ID(I), ID(IB)}, {ID(O), ID(OB)}, false);
	ct->setup_type(ID(GTP_INBUFG), {ID(I)}, {ID(O)}, false);
	ct->setup_type(ID(GTP_INBUFGDS), {ID(I), ID(IB)}, {ID(O)}, false);
	ct->setup_type(ID(GTP_INV), {ID(I)}, {ID(Z)}, false);
	ct->setup_type(ID(GTP_IOBUF), {ID(I), ID(T)}, {ID(O)}, false);
	ct->setup_type(ID(GTP_IOBUF_RX_MIPI), {ID(I_LP), ID(IB_LP), ID(T), ID(TB), ID(M)}, {ID(O_LP), ID(OB_LP), ID(O_HS)}, false);
	ct->setup_type(ID(GTP_IOBUF_TX_MIPI), {ID(I_LP), ID(IB_LP), ID(I_HS), ID(T), ID(TB), ID(M)}, {ID(O_LP), ID(OB_LP)}, false);
	ct->setup_type(ID(GTP_IOBUFCO), {ID(I), ID(T)}, {ID(O)}, false);
	ct->setup_type(ID(GTP_IOBUFCO_E1), {ID(I), ID(IB), ID(T), ID(TB)}, {ID(O)}, false);
	ct->setup_type(ID(GTP_IOBUFDS), {ID(I), ID(T)}, {ID(O)}, false);
	ct->setup_type(ID(GTP_IOBUFE), {ID(I), ID(EN), ID(T)}, {ID(O)}, false);
	ct->setup_type(ID(GTP_IOBUFECO), {ID(I), ID(EN), ID(T)}, {ID(O)}, false);
	ct->setup_type(ID(GTP_IOBUFEDS), {ID(I), ID(EN), ID(T)}, {ID(O)}, false);
	ct->setup_type(ID(GTP_IOCLKBUF), {ID(CLKIN), ID(DI)}, {ID(CLKOUT)}, false);
	ct->setup_type(ID(GTP_IOCLKDIV_E2), {ID(CLKIN), ID(RST_N), ID(CE)}, {ID(CLKDIVOUT)}, false);
	ct->setup_type(ID(GTP_IOCLKDIV_E3), {ID(RST), ID(CLKIN)}, {ID(CLKDIVOUT)}, false);
	ct->setup_type(ID(GTP_IODELAY_E2), {ID(DI), ID(DELAY_SEL), ID(DELAY_STEP), ID(EN_N)}, {ID(DO)}, false);
	ct->setup_type(ID(GTP_IPAL_E2), {ID(RST_N), ID(CLK), ID(CS_N), ID(RW_SEL), ID(DI)},
		       {ID(DO), ID(RBCRC_ERR), ID(RBCRC_VALID), ID(ECC_VALID), ID(ECC_INDEX), ID(SERROR), ID(DERROR), ID(SEU_FRAME_ADDR),
			ID(SEU_COLUMN_ADDR), ID(SEU_REGION_ADDR), ID(SEU_FRAME_NADDR), ID(SEU_COLUMN_NADDR), ID(SEU_REGION_NADDR), ID(PRCFG_OVER),
			ID(PRCFG_ERR), ID(DRCFG_OVER), ID(DRCFG_ERR)},
		       false);
	ct->setup_type(ID(GTP_ISERDES_E2),
		       {ID(RST), ID(ICE0), ID(ICE1), ID(DESCLK), ID(ICLK), ID(ICLKB), ID(OCLK), ID(ICLKDIV), ID(DI), ID(BITSLIP), ID(ISHIFTIN0),
			ID(ISHIFTIN1), ID(IFIFO_WADDR), ID(IFIFO_RADDR)},
		       {ID(DO), ID(ISHIFTOUT0), ID(ISHIFTOUT1)}, false);
	ct->setup_type(ID(GTP_ISERDES_FIFO), {ID(DIN), ID(VALID_I), ID(RST), ID(EN), ID(WCLK), ID(RCLK)},
		       {ID(DOUT), ID(VALID_O), ID(EMPTY), ID(FULL)}, false);
	ct->setup_type(ID(GTP_JTAGIF), {ID(TCK), ID(TMS), ID(TDI)}, {ID(TDO)}, false);
	ct->setup_type(ID(GTP_KEYRAM), {ID(ERASE_KEY_N)}, {}, false);
	ct->setup_type(ID(GTP_LUT1), {ID(I0)}, {ID(Z)}, false);
	ct->setup_type(ID(GTP_LUT2), {ID(I0), ID(I1)}, {ID(Z)}, false);
	ct->setup_type(ID(GTP_LUT3), {ID(I0), ID(I1), ID(I2)}, {ID(Z)}, false);
	ct->setup_type(ID(GTP_LUT4), {ID(I0), ID(I1), ID(I2), ID(I3)}, {ID(Z)}, false);
	ct->setup_type(ID(GTP_LUT5), {ID(I0), ID(I1), ID(I2), ID(I3), ID(I4)}, {ID(Z)}, false);
	ct->setup_type(ID(GTP_LUT6), {ID(I0), ID(I1), ID(I2), ID(I3), ID(I4), ID(I5)}, {ID(Z)}, false);
	ct->setup_type(ID(GTP_LUT6CARRY), {ID(CIN), ID(I0), ID(I1), ID(I2), ID(I3), ID(I4), ID(I5)}, {ID(COUT), ID(Z)}, false);
	ct->setup_type(ID(GTP_LUT6D), {ID(I0), ID(I1), ID(I2), ID(I3), ID(I4), ID(I5)}, {ID(Z), ID(Z5)}, false);
	ct->setup_type(ID(GTP_LUT7), {ID(I0), ID(I1), ID(I2), ID(I3), ID(I4), ID(I5), ID(I6)}, {ID(Z)}, false);
	ct->setup_type(ID(GTP_LUT8), {ID(I0), ID(I1), ID(I2), ID(I3), ID(I4), ID(I5), ID(I6), ID(I7)}, {ID(Z)}, false);
	ct->setup_type(ID(GTP_MONITOR_E1), {ID(RST_N), ID(CLK), ID(EN), ID(SAMPLE), ID(SEL)}, {ID(DATA), ID(DATA_VALID), ID(READY)}, false);
	ct->setup_type(ID(GTP_MONITOR_E1_DFT),
		       {ID(RST_N), ID(CLK), ID(EN), ID(SAMPLE), ID(SEL), ID(TEST_BIST_MODE), ID(TEST_BIST_START), ID(TEST_TDI), ID(TEST_TCK),
			ID(TEST_FLG_JTAG), ID(TEST_SHIFTDR), ID(TEST_CLOCKDR), ID(TEST_CAPTUREDR), ID(TEST_UPDATEDR), ID(TEST_BIST_REF_MAX),
			ID(TEST_BIST_REF_MIN)},
		       {ID(DATA), ID(DATA_VALID), ID(READY), ID(TEST_TDO), ID(TEST_BIST_VALID), ID(TEST_BIST_ERROR)}, false);
	ct->setup_type(ID(GTP_MUX2LUT7), {ID(I0), ID(I1), ID(S)}, {ID(Z)}, false);
	ct->setup_type(ID(GTP_MUX2LUT8), {ID(I0), ID(I1), ID(S)}, {ID(Z)}, false);
	ct->setup_type(ID(GTP_ODDR_E1), {ID(D0), ID(D1), ID(CLK), ID(CE), ID(RS)}, {ID(Q)}, false);
	ct->setup_type(ID(GTP_ONE), {}, {ID(Z)}, false);
	ct->setup_type(ID(GTP_OSC_E4), {ID(EN_N)}, {ID(CLKOUT)}, false);
	ct->setup_type(ID(GTP_OSERDES_E2),
		       {ID(RST), ID(OCE), ID(TCE), ID(OCLKDIV), ID(SERCLK), ID(OCLK), ID(MIPI_CTRL), ID(UPD0_SHIFT), ID(UPD1_SHIFT), ID(OSHIFTIN0),
			ID(OSHIFTIN1), ID(DI), ID(TI), ID(TBYTE_IN)},
		       {ID(OSHIFTOUT0), ID(OSHIFTOUT1), ID(TQ), ID(DO), ID(TFB), ID(TERM_FB)}, false);
	ct->setup_type(ID(GTP_OSERDES_FIFO), {ID(DIN), ID(TIN), ID(RST), ID(EN), ID(WCLK), ID(RCLK)}, {ID(DOUT), ID(TOUT), ID(EMPTY), ID(FULL)},
		       false);
	ct->setup_type(ID(GTP_OUTBUF), {ID(I)}, {ID(O)}, false);
	ct->setup_type(ID(GTP_OUTBUFCO), {ID(I)}, {ID(O), ID(OB)}, false);
	ct->setup_type(ID(GTP_OUTBUFCO_E1), {ID(I), ID(IB)}, {ID(O), ID(OB)}, false);
	ct->setup_type(ID(GTP_OUTBUFDS), {ID(I)}, {ID(O), ID(OB)}, false);
	ct->setup_type(ID(GTP_OUTBUFT), {ID(I), ID(T)}, {ID(O)}, false);
	ct->setup_type(ID(GTP_OUTBUFTCO), {ID(I), ID(T)}, {ID(O), ID(OB)}, false);
	ct->setup_type(ID(GTP_OUTBUFTCO_E1), {ID(I), ID(IB), ID(T), ID(TB)}, {ID(O), ID(OB)}, false);
	ct->setup_type(ID(GTP_OUTBUFTDS), {ID(I), ID(T)}, {ID(O), ID(OB)}, false);
	ct->setup_type(ID(GTP_PCIEGEN2),
		       {ID(MEM_CLK),
			ID(PCLK),
			ID(PCLK_DIV2),
			ID(BUTTON_RST),
			ID(POWER_UP_RST),
			ID(PERST),
			ID(APP_INIT_RST),
			ID(DEVICE_TYPE),
			ID(RX_LANE_FLIP_EN),
			ID(TX_LANE_FLIP_EN),
			ID(APP_LTSSM_EN),
			ID(APP_REQ_RETRY_EN),
			ID(AXIS_MASTER_TREADY),
			ID(TRGT1_RADM_PKT_HALT),
			ID(AXIS_SLAVE0_TVALID),
			ID(AXIS_SLAVE0_TDATA),
			ID(AXIS_SLAVE0_TLAST),
			ID(AXIS_SLAVE0_TUSER),
			ID(AXIS_SLAVE1_TVALID),
			ID(AXIS_SLAVE1_TDATA),
			ID(AXIS_SLAVE1_TLAST),
			ID(AXIS_SLAVE1_TUSER),
			ID(AXIS_SLAVE2_TVALID),
			ID(AXIS_SLAVE2_TDATA),
			ID(AXIS_SLAVE2_TLAST),
			ID(AXIS_SLAVE2_TUSER),
			ID(DBI_ADDR),
			ID(DBI_DIN),
			ID(DBI_CS),
			ID(DBI_CS2),
			ID(DBI_WR),
			ID(APP_DBI_RO_WR_DISABLE),
			ID(SEDI),
			ID(SEDI_ACK),
			ID(SYS_INT),
			ID(VEN_MSI_REQ),
			ID(VEN_MSI_TC),
			ID(VEN_MSI_VECTOR),
			ID(CFG_MSI_PENDING),
			ID(MSIX_ADDR),
			ID(MSIX_DATA),
			ID(OUTBAND_PWRUP_CMD),
			ID(APP_REQ_ENTR_L1),
			ID(APP_READY_ENTR_L23),
			ID(APP_REQ_EXIT_L1),
			ID(APP_XFER_PENDING),
			ID(APPS_PM_XMT_TURNOFF),
			ID(APP_UNLOCK_MSG),
			ID(APPS_PM_XMT_PME),
			ID(APP_CLK_PM_EN),
			ID(SYS_AUX_PWR_DET),
			ID(APP_HDR_VALID),
			ID(APP_HDR_LOG),
			ID(APP_ERR_BUS),
			ID(APP_ERR_ADVISORY),
			ID(DIAG_CTRL_BUS),
			ID(DYN_DEBUG_INFO_SEL),
			ID(APP_RAS_DES_SD_HOLD_LTSSM),
			ID(APP_RAS_DES_TBA_CTRL),
			ID(PHY_MAC_RXELECIDLE),
			ID(PHY_MAC_PHYSTATUS),
			ID(PHY_MAC_RXDATA),
			ID(PHY_MAC_RXDATAK),
			ID(PHY_MAC_RXVALID),
			ID(PHY_MAC_RXSTATUS),
			ID(P_DATAQ_DATAOUT),
			ID(RETRYRAM_XDLH_DATA),
			ID(P_HDRQ_DATAOUT),
			ID(RAM_TEST_EN),
			ID(RAM_TEST_ADDRH),
			ID(RETRY_TEST_DATA_EN),
			ID(RAM_TEST_MODE_N)},
		       {ID(CORE_RST_N),
			ID(TRAINING_RST_N),
			ID(PHY_RST_N),
			ID(SMLH_LINK_UP),
			ID(RDLH_LINK_UP),
			ID(SMLH_LTSSM_STATE),
			ID(AXIS_MASTER_TVALID),
			ID(AXIS_MASTER_TDATA),
			ID(AXIS_MASTER_TKEEP),
			ID(AXIS_MASTER_TLAST),
			ID(AXIS_MASTER_TUSER),
			ID(RADM_GRANT_TLP_TYPE),
			ID(AXIS_SLAVE0_TREADY),
			ID(AXIS_SLAVE1_TREADY),
			ID(AXIS_SLAVE2_TREADY),
			ID(PM_XTLH_BLOCK_TLP),
			ID(LBC_DBI_ACK),
			ID(LBC_DBI_DOUT),
			ID(SEDO),
			ID(SEDO_EN),
			ID(CFG_INT_DISABLE),
			ID(INTA_GRT_MUX),
			ID(INTB_GRT_MUX),
			ID(INTC_GRT_MUX),
			ID(INTD_GRT_MUX),
			ID(VEN_MSI_GRANT),
			ID(CFG_MSI_EN),
			ID(CFG_MSIX_EN),
			ID(CFG_MSIX_FUNC_MASK),
			ID(RADM_PM_TURNOFF),
			ID(RADM_MSG_UNLOCK),
			ID(PM_STATUS),
			ID(PM_DSTATE),
			ID(AUX_PM_EN),
			ID(PM_PME_EN),
			ID(PM_LINKST_IN_L0S),
			ID(PM_LINKST_IN_L1),
			ID(PM_LINKST_IN_L2),
			ID(PM_LINKST_L2_EXIT),
			ID(WAKE),
			ID(RADM_PM_PME),
			ID(RADM_PM_TO_ACK),
			ID(PM_MASTER_STATE),
			ID(PM_SLAVE_STATE),
			ID(CFG_SEND_COR_ERR_MUX),
			ID(CFG_SEND_NF_ERR_MUX),
			ID(CFG_SEND_F_ERR_MUX),
			ID(CFG_SYS_ERR_RC),
			ID(CFG_AER_RC_ERR_MUX),
			ID(RADM_CPL_TIMEOUT),
			ID(RADM_TIMEOUT_CPL_TC),
			ID(RADM_TIMEOUT_CPL_TAG),
			ID(RADM_TIMEOUT_CPL_ATTR),
			ID(RADM_TIMEOUT_CPL_LEN),
			ID(CFG_MAX_RD_REQ_SIZE),
			ID(CFG_BUS_MASTER_EN),
			ID(CFG_MAX_PAYLOAD_SIZE),
			ID(CFG_RCB),
			ID(CFG_MEM_SPACE_EN),
			ID(CFG_PM_NO_SOFT_RST),
			ID(CFG_CRS_SW_VIS_EN),
			ID(CFG_NO_SNOOP_EN),
			ID(CFG_RELAX_ORDER_EN),
			ID(CFG_TPH_REQ_EN),
			ID(CFG_PF_TPH_ST_MODE),
			ID(CFG_PBUS_NUM),
			ID(CFG_PBUS_DEV_NUM),
			ID(RBAR_CTRL_UPDATE),
			ID(CFG_ATOMIC_REQ_EN),
			ID(CFG_ATOMIC_EGRESS_BLOCK),
			ID(CFG_EXT_TAG_EN),
			ID(RADM_IDLE),
			ID(RADM_Q_NOT_EMPTY),
			ID(RADM_QOVERFLOW),
			ID(CFG_LINK_AUTO_BW_MUX),
			ID(CFG_BW_MGT_MUX),
			ID(CFG_PME_MUX),
			ID(DEBUG_INFO_MUX),
			ID(CFG_IDO_REQ_EN),
			ID(CFG_IDO_CPL_EN),
			ID(XADM_PH_CDTS),
			ID(XADM_PD_CDTS),
			ID(XADM_NPH_CDTS),
			ID(XADM_NPD_CDTS),
			ID(XADM_CPLH_CDTS),
			ID(XADM_CPLD_CDTS),
			ID(MAC_PHY_POWERDOWN),
			ID(MAC_PHY_TXDATA),
			ID(MAC_PHY_TXDATAK),
			ID(MAC_PHY_TXDETECTRX_LOOPBACK),
			ID(MAC_PHY_TXELECIDLE_L),
			ID(MAC_PHY_TXELECIDLE_H),
			ID(MAC_PHY_TXCOMPLIANCE),
			ID(MAC_PHY_RXPOLARITY),
			ID(MAC_PHY_RATE),
			ID(MAC_PHY_TXDEEMPH),
			ID(MAC_PHY_TXMARGIN),
			ID(MAC_PHY_TXSWING),
			ID(CFG_HW_AUTO_SP_DIS),
			ID(P_DATAQ_ADDRA),
			ID(P_DATAQ_ADDRB),
			ID(P_DATAQ_DATAIN),
			ID(P_DATAQ_ENA),
			ID(P_DATAQ_ENB),
			ID(P_DATAQ_WEA),
			ID(XDLH_RETRYRAM_ADDR),
			ID(XDLH_RETRYRAM_DATA),
			ID(XDLH_RETRYRAM_WE),
			ID(XDLH_RETRYRAM_EN),
			ID(P_HDRQ_ADDRA),
			ID(P_HDRQ_ADDRB),
			ID(P_HDRQ_DATAIN),
			ID(P_HDRQ_ENA),
			ID(P_HDRQ_ENB),
			ID(P_HDRQ_WEA)},
		       false);
	ct->setup_type(ID(GTP_PCIEGEN2_DFT),
		       {ID(MEM_CLK),
			ID(PCLK),
			ID(PCLK_DIV2),
			ID(BUTTON_RST),
			ID(POWER_UP_RST),
			ID(PERST),
			ID(APP_INIT_RST),
			ID(DEVICE_TYPE),
			ID(RX_LANE_FLIP_EN),
			ID(TX_LANE_FLIP_EN),
			ID(APP_LTSSM_EN),
			ID(APP_REQ_RETRY_EN),
			ID(AXIS_MASTER_TREADY),
			ID(TRGT1_RADM_PKT_HALT),
			ID(AXIS_SLAVE0_TVALID),
			ID(AXIS_SLAVE0_TDATA),
			ID(AXIS_SLAVE0_TLAST),
			ID(AXIS_SLAVE0_TUSER),
			ID(AXIS_SLAVE1_TVALID),
			ID(AXIS_SLAVE1_TDATA),
			ID(AXIS_SLAVE1_TLAST),
			ID(AXIS_SLAVE1_TUSER),
			ID(AXIS_SLAVE2_TVALID),
			ID(AXIS_SLAVE2_TDATA),
			ID(AXIS_SLAVE2_TLAST),
			ID(AXIS_SLAVE2_TUSER),
			ID(DBI_ADDR),
			ID(DBI_DIN),
			ID(DBI_CS),
			ID(DBI_CS2),
			ID(DBI_WR),
			ID(APP_DBI_RO_WR_DISABLE),
			ID(SEDI),
			ID(SEDI_ACK),
			ID(SYS_INT),
			ID(VEN_MSI_REQ),
			ID(VEN_MSI_TC),
			ID(VEN_MSI_VECTOR),
			ID(CFG_MSI_PENDING),
			ID(MSIX_ADDR),
			ID(MSIX_DATA),
			ID(OUTBAND_PWRUP_CMD),
			ID(APP_REQ_ENTR_L1),
			ID(APP_READY_ENTR_L23),
			ID(APP_REQ_EXIT_L1),
			ID(APP_XFER_PENDING),
			ID(APPS_PM_XMT_TURNOFF),
			ID(APP_UNLOCK_MSG),
			ID(APPS_PM_XMT_PME),
			ID(APP_CLK_PM_EN),
			ID(SYS_AUX_PWR_DET),
			ID(APP_HDR_VALID),
			ID(APP_HDR_LOG),
			ID(APP_ERR_BUS),
			ID(APP_ERR_ADVISORY),
			ID(DIAG_CTRL_BUS),
			ID(DYN_DEBUG_INFO_SEL),
			ID(APP_RAS_DES_SD_HOLD_LTSSM),
			ID(APP_RAS_DES_TBA_CTRL),
			ID(PHY_MAC_RXELECIDLE),
			ID(PHY_MAC_PHYSTATUS),
			ID(PHY_MAC_RXDATA),
			ID(PHY_MAC_RXDATAK),
			ID(PHY_MAC_RXVALID),
			ID(PHY_MAC_RXSTATUS),
			ID(P_DATAQ_DATAOUT),
			ID(RETRYRAM_XDLH_DATA),
			ID(P_HDRQ_DATAOUT),
			ID(RAM_TEST_EN),
			ID(RAM_TEST_ADDRH),
			ID(RETRY_TEST_DATA_EN),
			ID(RAM_TEST_MODE_N),
			ID(TEST_SE_N),
			ID(TEST_RST_N),
			ID(TEST_MODE_N)},
		       {ID(CORE_RST_N),
			ID(TRAINING_RST_N),
			ID(PHY_RST_N),
			ID(SMLH_LINK_UP),
			ID(RDLH_LINK_UP),
			ID(SMLH_LTSSM_STATE),
			ID(AXIS_MASTER_TVALID),
			ID(AXIS_MASTER_TDATA),
			ID(AXIS_MASTER_TKEEP),
			ID(AXIS_MASTER_TLAST),
			ID(AXIS_MASTER_TUSER),
			ID(RADM_GRANT_TLP_TYPE),
			ID(AXIS_SLAVE0_TREADY),
			ID(AXIS_SLAVE1_TREADY),
			ID(AXIS_SLAVE2_TREADY),
			ID(PM_XTLH_BLOCK_TLP),
			ID(LBC_DBI_ACK),
			ID(LBC_DBI_DOUT),
			ID(SEDO),
			ID(SEDO_EN),
			ID(CFG_INT_DISABLE),
			ID(INTA_GRT_MUX),
			ID(INTB_GRT_MUX),
			ID(INTC_GRT_MUX),
			ID(INTD_GRT_MUX),
			ID(VEN_MSI_GRANT),
			ID(CFG_MSI_EN),
			ID(CFG_MSIX_EN),
			ID(CFG_MSIX_FUNC_MASK),
			ID(RADM_PM_TURNOFF),
			ID(RADM_MSG_UNLOCK),
			ID(PM_STATUS),
			ID(PM_DSTATE),
			ID(AUX_PM_EN),
			ID(PM_PME_EN),
			ID(PM_LINKST_IN_L0S),
			ID(PM_LINKST_IN_L1),
			ID(PM_LINKST_IN_L2),
			ID(PM_LINKST_L2_EXIT),
			ID(WAKE),
			ID(RADM_PM_PME),
			ID(RADM_PM_TO_ACK),
			ID(PM_MASTER_STATE),
			ID(PM_SLAVE_STATE),
			ID(CFG_SEND_COR_ERR_MUX),
			ID(CFG_SEND_NF_ERR_MUX),
			ID(CFG_SEND_F_ERR_MUX),
			ID(CFG_SYS_ERR_RC),
			ID(CFG_AER_RC_ERR_MUX),
			ID(RADM_CPL_TIMEOUT),
			ID(RADM_TIMEOUT_CPL_TC),
			ID(RADM_TIMEOUT_CPL_TAG),
			ID(RADM_TIMEOUT_CPL_ATTR),
			ID(RADM_TIMEOUT_CPL_LEN),
			ID(CFG_MAX_RD_REQ_SIZE),
			ID(CFG_BUS_MASTER_EN),
			ID(CFG_MAX_PAYLOAD_SIZE),
			ID(CFG_RCB),
			ID(CFG_MEM_SPACE_EN),
			ID(CFG_PM_NO_SOFT_RST),
			ID(CFG_CRS_SW_VIS_EN),
			ID(CFG_NO_SNOOP_EN),
			ID(CFG_RELAX_ORDER_EN),
			ID(CFG_TPH_REQ_EN),
			ID(CFG_PF_TPH_ST_MODE),
			ID(CFG_PBUS_NUM),
			ID(CFG_PBUS_DEV_NUM),
			ID(RBAR_CTRL_UPDATE),
			ID(CFG_ATOMIC_REQ_EN),
			ID(CFG_ATOMIC_EGRESS_BLOCK),
			ID(CFG_EXT_TAG_EN),
			ID(RADM_IDLE),
			ID(RADM_Q_NOT_EMPTY),
			ID(RADM_QOVERFLOW),
			ID(CFG_LINK_AUTO_BW_MUX),
			ID(CFG_BW_MGT_MUX),
			ID(CFG_PME_MUX),
			ID(DEBUG_INFO_MUX),
			ID(CFG_IDO_REQ_EN),
			ID(CFG_IDO_CPL_EN),
			ID(XADM_PH_CDTS),
			ID(XADM_PD_CDTS),
			ID(XADM_NPH_CDTS),
			ID(XADM_NPD_CDTS),
			ID(XADM_CPLH_CDTS),
			ID(XADM_CPLD_CDTS),
			ID(MAC_PHY_POWERDOWN),
			ID(MAC_PHY_TXDATA),
			ID(MAC_PHY_TXDATAK),
			ID(MAC_PHY_TXDETECTRX_LOOPBACK),
			ID(MAC_PHY_TXELECIDLE_L),
			ID(MAC_PHY_TXELECIDLE_H),
			ID(MAC_PHY_TXCOMPLIANCE),
			ID(MAC_PHY_RXPOLARITY),
			ID(MAC_PHY_RATE),
			ID(MAC_PHY_TXDEEMPH),
			ID(MAC_PHY_TXMARGIN),
			ID(MAC_PHY_TXSWING),
			ID(CFG_HW_AUTO_SP_DIS),
			ID(P_DATAQ_ADDRA),
			ID(P_DATAQ_ADDRB),
			ID(P_DATAQ_DATAIN),
			ID(P_DATAQ_ENA),
			ID(P_DATAQ_ENB),
			ID(P_DATAQ_WEA),
			ID(XDLH_RETRYRAM_ADDR),
			ID(XDLH_RETRYRAM_DATA),
			ID(XDLH_RETRYRAM_WE),
			ID(XDLH_RETRYRAM_EN),
			ID(P_HDRQ_ADDRA),
			ID(P_HDRQ_ADDRB),
			ID(P_HDRQ_DATAIN),
			ID(P_HDRQ_ENA),
			ID(P_HDRQ_ENB),
			ID(P_HDRQ_WEA)},
		       false);
	ct->setup_type(ID(GTP_PCIEGEN3),
		       {ID(PCLK),
			ID(PCLK_DIV2),
			ID(MEM_CLK),
			ID(USER_CLK),
			ID(BUTTON_RST),
			ID(POWER_UP_RST),
			ID(PERST),
			ID(APP_INIT_RST),
			ID(DEVICE_TYPE),
			ID(RX_LANE_FLIP_EN),
			ID(TX_LANE_FLIP_EN),
			ID(APP_LTSSM_ENABLE),
			ID(APP_REQ_RETRY_EN),
			ID(APP_PF_REQ_RETRY_EN),
			ID(APP_VF_REQ_RETRY_EN),
			ID(AXIS_MASTER0_TREADY),
			ID(USER_RCVD_NP_READY),
			ID(USER_RCVD_P_READY),
			ID(AXIS_SLAVE0_TDATA),
			ID(AXIS_SLAVE0_TLAST),
			ID(AXIS_SLAVE0_TUSER),
			ID(AXIS_SLAVE0_TVALID),
			ID(AXIS_SLAVE1_TDATA),
			ID(AXIS_SLAVE1_TLAST),
			ID(AXIS_SLAVE1_TUSER),
			ID(AXIS_SLAVE1_TVALID),
			ID(AXIS_SLAVE2_TDATA),
			ID(AXIS_SLAVE2_TLAST),
			ID(AXIS_SLAVE2_TUSER),
			ID(AXIS_SLAVE2_TVALID),
			ID(APP_DBI_RO_WR_DISABLE),
			ID(DBI_ADDR),
			ID(DBI_CS),
			ID(DBI_CS2),
			ID(DBI_DIN),
			ID(DBI_FUNC_NUM),
			ID(DBI_VFUNC_ACTIVE),
			ID(DBI_VFUNC_NUM),
			ID(DBI_WR),
			ID(SEDI),
			ID(SEDI_ACK),
			ID(SYS_INT),
			ID(CFG_MSI_PENDING),
			ID(CFG_VF_MSI_PENDING),
			ID(VEN_MSI_FUNC_NUM),
			ID(VEN_MSI_REQ),
			ID(VEN_MSI_TC),
			ID(VEN_MSI_VECTOR),
			ID(VEN_MSI_VFUNC_ACTIVE),
			ID(VEN_MSI_VFUNC_NUM),
			ID(MSIX_ADDR),
			ID(MSIX_DATA),
			ID(APP_CLK_PM_EN),
			ID(APPS_PM_VF_XMT_PME),
			ID(APPS_PM_XMT_PME),
			ID(APPS_PM_XMT_TURNOFF),
			ID(APP_UNLOCK_MSG),
			ID(APP_READY_ENTR_L23),
			ID(APP_REQ_ENTR_L1),
			ID(APP_REQ_EXIT_L1),
			ID(CFG_PWR_BUDGET_DATA_REG),
			ID(CFG_PWR_BUDGET_FUNC_NUM),
			ID(CFG_PWR_BUDGET_VALID),
			ID(APP_XFER_PENDING),
			ID(APP_HDR_LOG),
			ID(APP_HDR_VALID),
			ID(APP_ERR_ADVISORY),
			ID(APP_ERR_BUS),
			ID(APP_ERR_FUNC_NUM),
			ID(APP_ERR_VFUNC_ACTIVE),
			ID(APP_ERR_VFUNC_NUM),
			ID(APP_LTR_MSG_FUNC_NUM),
			ID(APP_LTR_MSG_LATENCY),
			ID(APP_LTR_MSG_REQ),
			ID(APP_FLR_PF_DONE),
			ID(APP_FLR_VF_DONE),
			ID(APP_OBFF_CPU_ACTIVE_MSG_REQ),
			ID(APP_OBFF_IDLE_MSG_REQ),
			ID(APP_OBFF_OBFF_MSG_REQ),
			ID(APP_RAS_DES_SD_HOLD_LTSSM),
			ID(DIAG_CTRL_BUS),
			ID(DYN_DEBUG_INFO_SEL),
			ID(PHY_MAC_DIRFEEDBACK),
			ID(PHY_MAC_FOMFEEDBACK),
			ID(PHY_MAC_LOCALFS),
			ID(PHY_MAC_LOCALLF),
			ID(PHY_MAC_LOCAL_TX_COEF_VALID),
			ID(PHY_MAC_LOCAL_TX_PSET_COEF),
			ID(PHY_MAC_PHYSTATUS),
			ID(PHY_MAC_RXDATA),
			ID(PHY_MAC_RXDATAK),
			ID(PHY_MAC_RXDATAVALID),
			ID(PHY_MAC_RXELECIDLE),
			ID(PHY_MAC_RXSTARTBLOCK),
			ID(PHY_MAC_RXSTATUS),
			ID(PHY_MAC_RXSYNCHEADER),
			ID(PHY_MAC_RXVALID),
			ID(PNP_RAM_RD_DATA),
			ID(RETRYRAM_XDLH_DATA),
			ID(TPH_RD_DATA_VALID),
			ID(TPH_RAM_RD_DATA),
			ID(RAM_TEST_EN),
			ID(RAM_TEST_ADDRH),
			ID(RETRY_TEST_DATA_EN),
			ID(RAM_TEST_MODE_N)},
		       {ID(USER_RST_N),
			ID(TRAINING_RST_N),
			ID(PHY_RST_N),
			ID(SMLH_LINK_UP),
			ID(RDLH_LINK_UP),
			ID(SMLH_LTSSM_STATE),
			ID(CFG_2ND_RESET),
			ID(LINK_REQ_RST),
			ID(CFG_VF_BME),
			ID(SMLH_REQ_RST),
			ID(AXIS_MASTER0_TDATA),
			ID(AXIS_MASTER0_TKEEP),
			ID(AXIS_MASTER0_TLAST),
			ID(AXIS_MASTER0_TUSER),
			ID(AXIS_MASTER0_TVALID),
			ID(CORE_AVL_NP_CNT),
			ID(CORE_AVL_P_CNT),
			ID(AXIS_MASTER1_TDATA),
			ID(AXIS_MASTER1_TKEEP),
			ID(AXIS_MASTER1_TUSER),
			ID(AXIS_MASTER1_TVALID),
			ID(AXIS_SLAVE0_TREADY),
			ID(AXIS_SLAVE1_TREADY),
			ID(AXIS_SLAVE2_TREADY),
			ID(RADM_CPL_TIMEOUT),
			ID(RADM_TIMEOUT_CPL_ATTR),
			ID(RADM_TIMEOUT_CPL_LEN),
			ID(RADM_TIMEOUT_CPL_TAG),
			ID(RADM_TIMEOUT_CPL_TC),
			ID(RADM_TIMEOUT_FUNC_NUM),
			ID(RADM_TIMEOUT_VFUNC_ACTIVE),
			ID(RADM_TIMEOUT_VFUNC_NUM),
			ID(LBC_DBI_ACK),
			ID(LBC_DBI_DOUT),
			ID(SEDO),
			ID(SEDO_EN),
			ID(CFG_INT_DISABLE),
			ID(INT_GRT),
			ID(CFG_MSI_EN),
			ID(CFG_MSI_MASK_UPDATE),
			ID(CFG_MULTI_MSI_EN),
			ID(CFG_VF_MSI_EN),
			ID(CFG_VF_MULTI_MSI_EN),
			ID(VEN_MSI_GRANT),
			ID(CFG_MSIX_EN),
			ID(CFG_MSIX_FUNC_MASK),
			ID(CFG_VF_MSIX_EN),
			ID(CFG_VF_MSIX_FUNC_MASK),
			ID(CFG_BW_MGT_MSI),
			ID(CFG_LINK_AUTO_BW_MSI),
			ID(CFG_PME_MSI),
			ID(CFG_BW_MGT_INT),
			ID(CFG_LINK_AUTO_BW_INT),
			ID(CFG_LINK_EQ_REQ_INT),
			ID(CFG_PME_INT),
			ID(CFG_NF_ERR_RPT_EN),
			ID(CFG_NO_SNOOP_EN),
			ID(CFG_OBFF_EN),
			ID(CFG_PBUS_DEV_NUM),
			ID(CFG_PBUS_NUM),
			ID(CFG_MEM_SPACE_EN),
			ID(CFG_EXT_TAG_EN),
			ID(CFG_F_ERR_RPT_EN),
			ID(CFG_ARI_FWD_EN),
			ID(CFG_ATOMIC_EGRESS_BLOCK),
			ID(CFG_ATOMIC_REQ_EN),
			ID(CFG_BUS_MASTER_EN),
			ID(CFG_COR_ERR_RPT_EN),
			ID(CFG_CRS_SW_VIS_EN),
			ID(CFG_MAX_PAYLOAD_SIZE),
			ID(CFG_MAX_RD_REQ_SIZE),
			ID(CFG_VF_EN),
			ID(CFG_TC_ENABLE),
			ID(CFG_RCB),
			ID(CFG_REG_SERREN),
			ID(CFG_RELAX_ORDER_EN),
			ID(RBAR_CTRL_UPDATE),
			ID(AUX_PM_EN),
			ID(PM_DSTATE),
			ID(PM_MASTER_STATE),
			ID(PM_PME_EN),
			ID(PM_SLAVE_STATE),
			ID(PM_STATUS),
			ID(PM_VF_DSTATE),
			ID(PM_VF_PME_EN),
			ID(PM_VF_STATUS),
			ID(PM_XTLH_BLOCK_TLP),
			ID(DPA_SUBSTATE_UPDATE),
			ID(WAKE),
			ID(CFG_PWR_BUDGET_DATA_SEL_REG),
			ID(CFG_PWR_BUDGET_SEL),
			ID(CFG_SEND_COR_ERR),
			ID(CFG_SEND_F_ERR),
			ID(CFG_SEND_NF_ERR),
			ID(CFG_AER_RC_ERR_INT),
			ID(CFG_AER_RC_ERR_MSI),
			ID(CFG_SYS_ERR_RC),
			ID(APP_LTR_MSG_GRANT),
			ID(CFG_LTR_M_EN),
			ID(CFG_DISABLE_LTR_CLR_MSG),
			ID(CFG_FLR_PF_ACTIVE),
			ID(CFG_FLR_VF_ACTIVE),
			ID(CFG_START_VFI),
			ID(CFG_NUM_VF),
			ID(APP_OBFF_MSG_GRANT),
			ID(MSG_RCVD),
			ID(MSG_RCVD_DATA),
			ID(MSG_RCVD_TYPE),
			ID(DEBUG_INFO_MUX),
			ID(CFG_IDO_CPL_EN),
			ID(CFG_IDO_REQ_EN),
			ID(XADM_CPLD_CDTS),
			ID(XADM_CPLH_CDTS),
			ID(XADM_NPD_CDTS),
			ID(XADM_NPH_CDTS),
			ID(XADM_PD_CDTS),
			ID(XADM_PH_CDTS),
			ID(RADM_Q_NOT_EMPTY),
			ID(RADM_QOVERFLOW),
			ID(MAC_PHY_BLOCKALIGNCONTROL),
			ID(MAC_PHY_DIRCHANGE),
			ID(MAC_PHY_FS),
			ID(MAC_PHY_GETLOCAL_PSET_COEF),
			ID(MAC_PHY_INVALID_REQ),
			ID(MAC_PHY_LF),
			ID(MAC_PHY_LOCAL_PSET_INDEX),
			ID(MAC_PHY_POWERDOWN),
			ID(MAC_PHY_RATE),
			ID(MAC_PHY_RXEQEVAL),
			ID(MAC_PHY_RXEQINPROGRESS),
			ID(MAC_PHY_RXPOLARITY),
			ID(MAC_PHY_RXPRESETHINT),
			ID(MAC_PHY_TXCOMPLIANCE),
			ID(MAC_PHY_TXDATA),
			ID(MAC_PHY_TXDATAK),
			ID(MAC_PHY_TXDATAVALID),
			ID(MAC_PHY_TXDEEMPH),
			ID(MAC_PHY_TXDETECTRX_LOOPBACK),
			ID(MAC_PHY_TXELECIDLE_H),
			ID(MAC_PHY_TXELECIDLE_L),
			ID(MAC_PHY_TXMARGIN),
			ID(MAC_PHY_TXSTARTBLOCK),
			ID(MAC_PHY_TXSWING),
			ID(MAC_PHY_TXSYNCHEADER),
			ID(PNP_RAM_RD_ADDR),
			ID(PNP_RAM_RD_EN),
			ID(PNP_RAM_WR_ADDR),
			ID(PNP_RAM_WR_DATA),
			ID(PNP_RAM_WR_EN),
			ID(XDLH_RETRYRAM_ADDR),
			ID(XDLH_RETRYRAM_DATA),
			ID(XDLH_RETRYRAM_EN),
			ID(XDLH_RETRYRAM_WE),
			ID(CFG_PF_TPH_ST_MODE),
			ID(CFG_TPH_REQ_EN),
			ID(CFG_VF_TPH_REQ_EN),
			ID(CFG_VF_TPH_ST_MODE),
			ID(TPH_RAM_ADDR),
			ID(TPH_RAM_FUNC_NUM),
			ID(TPH_RAM_FUNC_ACTIVE),
			ID(TPH_RAM_WR_BYTE_EN),
			ID(TPH_RAM_WR_DATA),
			ID(TPH_RAM_WR_EN)},
		       false);
	ct->setup_type(ID(GTP_PCIEGEN3_DFT),
		       {ID(PCLK),
			ID(PCLK_DIV2),
			ID(MEM_CLK),
			ID(USER_CLK),
			ID(BUTTON_RST),
			ID(POWER_UP_RST),
			ID(PERST),
			ID(APP_INIT_RST),
			ID(DEVICE_TYPE),
			ID(RX_LANE_FLIP_EN),
			ID(TX_LANE_FLIP_EN),
			ID(APP_LTSSM_ENABLE),
			ID(APP_REQ_RETRY_EN),
			ID(APP_PF_REQ_RETRY_EN),
			ID(APP_VF_REQ_RETRY_EN),
			ID(AXIS_MASTER0_TREADY),
			ID(USER_RCVD_NP_READY),
			ID(USER_RCVD_P_READY),
			ID(AXIS_SLAVE0_TDATA),
			ID(AXIS_SLAVE0_TLAST),
			ID(AXIS_SLAVE0_TUSER),
			ID(AXIS_SLAVE0_TVALID),
			ID(AXIS_SLAVE1_TDATA),
			ID(AXIS_SLAVE1_TLAST),
			ID(AXIS_SLAVE1_TUSER),
			ID(AXIS_SLAVE1_TVALID),
			ID(AXIS_SLAVE2_TDATA),
			ID(AXIS_SLAVE2_TLAST),
			ID(AXIS_SLAVE2_TUSER),
			ID(AXIS_SLAVE2_TVALID),
			ID(APP_DBI_RO_WR_DISABLE),
			ID(DBI_ADDR),
			ID(DBI_CS),
			ID(DBI_CS2),
			ID(DBI_DIN),
			ID(DBI_FUNC_NUM),
			ID(DBI_VFUNC_ACTIVE),
			ID(DBI_VFUNC_NUM),
			ID(DBI_WR),
			ID(SEDI),
			ID(SEDI_ACK),
			ID(SYS_INT),
			ID(CFG_MSI_PENDING),
			ID(CFG_VF_MSI_PENDING),
			ID(VEN_MSI_FUNC_NUM),
			ID(VEN_MSI_REQ),
			ID(VEN_MSI_TC),
			ID(VEN_MSI_VECTOR),
			ID(VEN_MSI_VFUNC_ACTIVE),
			ID(VEN_MSI_VFUNC_NUM),
			ID(MSIX_ADDR),
			ID(MSIX_DATA),
			ID(APP_CLK_PM_EN),
			ID(APPS_PM_VF_XMT_PME),
			ID(APPS_PM_XMT_PME),
			ID(APPS_PM_XMT_TURNOFF),
			ID(APP_UNLOCK_MSG),
			ID(APP_READY_ENTR_L23),
			ID(APP_REQ_ENTR_L1),
			ID(APP_REQ_EXIT_L1),
			ID(CFG_PWR_BUDGET_DATA_REG),
			ID(CFG_PWR_BUDGET_FUNC_NUM),
			ID(CFG_PWR_BUDGET_VALID),
			ID(APP_XFER_PENDING),
			ID(APP_HDR_LOG),
			ID(APP_HDR_VALID),
			ID(APP_ERR_ADVISORY),
			ID(APP_ERR_BUS),
			ID(APP_ERR_FUNC_NUM),
			ID(APP_ERR_VFUNC_ACTIVE),
			ID(APP_ERR_VFUNC_NUM),
			ID(APP_LTR_MSG_FUNC_NUM),
			ID(APP_LTR_MSG_LATENCY),
			ID(APP_LTR_MSG_REQ),
			ID(APP_FLR_PF_DONE),
			ID(APP_FLR_VF_DONE),
			ID(APP_OBFF_CPU_ACTIVE_MSG_REQ),
			ID(APP_OBFF_IDLE_MSG_REQ),
			ID(APP_OBFF_OBFF_MSG_REQ),
			ID(APP_RAS_DES_SD_HOLD_LTSSM),
			ID(DIAG_CTRL_BUS),
			ID(DYN_DEBUG_INFO_SEL),
			ID(PHY_MAC_DIRFEEDBACK),
			ID(PHY_MAC_FOMFEEDBACK),
			ID(PHY_MAC_LOCALFS),
			ID(PHY_MAC_LOCALLF),
			ID(PHY_MAC_LOCAL_TX_COEF_VALID),
			ID(PHY_MAC_LOCAL_TX_PSET_COEF),
			ID(PHY_MAC_PHYSTATUS),
			ID(PHY_MAC_RXDATA),
			ID(PHY_MAC_RXDATAK),
			ID(PHY_MAC_RXDATAVALID),
			ID(PHY_MAC_RXELECIDLE),
			ID(PHY_MAC_RXSTARTBLOCK),
			ID(PHY_MAC_RXSTATUS),
			ID(PHY_MAC_RXSYNCHEADER),
			ID(PHY_MAC_RXVALID),
			ID(PNP_RAM_RD_DATA),
			ID(RETRYRAM_XDLH_DATA),
			ID(TPH_RD_DATA_VALID),
			ID(TPH_RAM_RD_DATA),
			ID(RAM_TEST_EN),
			ID(RAM_TEST_ADDRH),
			ID(RETRY_TEST_DATA_EN),
			ID(RAM_TEST_MODE_N),
			ID(TEST_MODE_N),
			ID(TEST_RST_N),
			ID(TEST_SE_N)},
		       {ID(USER_RST_N),
			ID(TRAINING_RST_N),
			ID(PHY_RST_N),
			ID(SMLH_LINK_UP),
			ID(RDLH_LINK_UP),
			ID(SMLH_LTSSM_STATE),
			ID(CFG_2ND_RESET),
			ID(LINK_REQ_RST),
			ID(CFG_VF_BME),
			ID(SMLH_REQ_RST),
			ID(AXIS_MASTER0_TDATA),
			ID(AXIS_MASTER0_TKEEP),
			ID(AXIS_MASTER0_TLAST),
			ID(AXIS_MASTER0_TUSER),
			ID(AXIS_MASTER0_TVALID),
			ID(CORE_AVL_NP_CNT),
			ID(CORE_AVL_P_CNT),
			ID(AXIS_MASTER1_TDATA),
			ID(AXIS_MASTER1_TKEEP),
			ID(AXIS_MASTER1_TUSER),
			ID(AXIS_MASTER1_TVALID),
			ID(AXIS_SLAVE0_TREADY),
			ID(AXIS_SLAVE1_TREADY),
			ID(AXIS_SLAVE2_TREADY),
			ID(RADM_CPL_TIMEOUT),
			ID(RADM_TIMEOUT_CPL_ATTR),
			ID(RADM_TIMEOUT_CPL_LEN),
			ID(RADM_TIMEOUT_CPL_TAG),
			ID(RADM_TIMEOUT_CPL_TC),
			ID(RADM_TIMEOUT_FUNC_NUM),
			ID(RADM_TIMEOUT_VFUNC_ACTIVE),
			ID(RADM_TIMEOUT_VFUNC_NUM),
			ID(LBC_DBI_ACK),
			ID(LBC_DBI_DOUT),
			ID(SEDO),
			ID(SEDO_EN),
			ID(CFG_INT_DISABLE),
			ID(INT_GRT),
			ID(CFG_MSI_EN),
			ID(CFG_MSI_MASK_UPDATE),
			ID(CFG_MULTI_MSI_EN),
			ID(CFG_VF_MSI_EN),
			ID(CFG_VF_MULTI_MSI_EN),
			ID(VEN_MSI_GRANT),
			ID(CFG_MSIX_EN),
			ID(CFG_MSIX_FUNC_MASK),
			ID(CFG_VF_MSIX_EN),
			ID(CFG_VF_MSIX_FUNC_MASK),
			ID(CFG_BW_MGT_MSI),
			ID(CFG_LINK_AUTO_BW_MSI),
			ID(CFG_PME_MSI),
			ID(CFG_BW_MGT_INT),
			ID(CFG_LINK_AUTO_BW_INT),
			ID(CFG_LINK_EQ_REQ_INT),
			ID(CFG_PME_INT),
			ID(CFG_NF_ERR_RPT_EN),
			ID(CFG_NO_SNOOP_EN),
			ID(CFG_OBFF_EN),
			ID(CFG_PBUS_DEV_NUM),
			ID(CFG_PBUS_NUM),
			ID(CFG_MEM_SPACE_EN),
			ID(CFG_EXT_TAG_EN),
			ID(CFG_F_ERR_RPT_EN),
			ID(CFG_ARI_FWD_EN),
			ID(CFG_ATOMIC_EGRESS_BLOCK),
			ID(CFG_ATOMIC_REQ_EN),
			ID(CFG_BUS_MASTER_EN),
			ID(CFG_COR_ERR_RPT_EN),
			ID(CFG_CRS_SW_VIS_EN),
			ID(CFG_MAX_PAYLOAD_SIZE),
			ID(CFG_MAX_RD_REQ_SIZE),
			ID(CFG_VF_EN),
			ID(CFG_TC_ENABLE),
			ID(CFG_RCB),
			ID(CFG_REG_SERREN),
			ID(CFG_RELAX_ORDER_EN),
			ID(RBAR_CTRL_UPDATE),
			ID(AUX_PM_EN),
			ID(PM_DSTATE),
			ID(PM_MASTER_STATE),
			ID(PM_PME_EN),
			ID(PM_SLAVE_STATE),
			ID(PM_STATUS),
			ID(PM_VF_DSTATE),
			ID(PM_VF_PME_EN),
			ID(PM_VF_STATUS),
			ID(PM_XTLH_BLOCK_TLP),
			ID(DPA_SUBSTATE_UPDATE),
			ID(WAKE),
			ID(CFG_PWR_BUDGET_DATA_SEL_REG),
			ID(CFG_PWR_BUDGET_SEL),
			ID(CFG_SEND_COR_ERR),
			ID(CFG_SEND_F_ERR),
			ID(CFG_SEND_NF_ERR),
			ID(CFG_AER_RC_ERR_INT),
			ID(CFG_AER_RC_ERR_MSI),
			ID(CFG_SYS_ERR_RC),
			ID(APP_LTR_MSG_GRANT),
			ID(CFG_LTR_M_EN),
			ID(CFG_DISABLE_LTR_CLR_MSG),
			ID(CFG_FLR_PF_ACTIVE),
			ID(CFG_FLR_VF_ACTIVE),
			ID(CFG_START_VFI),
			ID(CFG_NUM_VF),
			ID(APP_OBFF_MSG_GRANT),
			ID(MSG_RCVD),
			ID(MSG_RCVD_DATA),
			ID(MSG_RCVD_TYPE),
			ID(DEBUG_INFO_MUX),
			ID(CFG_IDO_CPL_EN),
			ID(CFG_IDO_REQ_EN),
			ID(XADM_CPLD_CDTS),
			ID(XADM_CPLH_CDTS),
			ID(XADM_NPD_CDTS),
			ID(XADM_NPH_CDTS),
			ID(XADM_PD_CDTS),
			ID(XADM_PH_CDTS),
			ID(RADM_Q_NOT_EMPTY),
			ID(RADM_QOVERFLOW),
			ID(MAC_PHY_BLOCKALIGNCONTROL),
			ID(MAC_PHY_DIRCHANGE),
			ID(MAC_PHY_FS),
			ID(MAC_PHY_GETLOCAL_PSET_COEF),
			ID(MAC_PHY_INVALID_REQ),
			ID(MAC_PHY_LF),
			ID(MAC_PHY_LOCAL_PSET_INDEX),
			ID(MAC_PHY_POWERDOWN),
			ID(MAC_PHY_RATE),
			ID(MAC_PHY_RXEQEVAL),
			ID(MAC_PHY_RXEQINPROGRESS),
			ID(MAC_PHY_RXPOLARITY),
			ID(MAC_PHY_RXPRESETHINT),
			ID(MAC_PHY_TXCOMPLIANCE),
			ID(MAC_PHY_TXDATA),
			ID(MAC_PHY_TXDATAK),
			ID(MAC_PHY_TXDATAVALID),
			ID(MAC_PHY_TXDEEMPH),
			ID(MAC_PHY_TXDETECTRX_LOOPBACK),
			ID(MAC_PHY_TXELECIDLE_H),
			ID(MAC_PHY_TXELECIDLE_L),
			ID(MAC_PHY_TXMARGIN),
			ID(MAC_PHY_TXSTARTBLOCK),
			ID(MAC_PHY_TXSWING),
			ID(MAC_PHY_TXSYNCHEADER),
			ID(PNP_RAM_RD_ADDR),
			ID(PNP_RAM_RD_EN),
			ID(PNP_RAM_WR_ADDR),
			ID(PNP_RAM_WR_DATA),
			ID(PNP_RAM_WR_EN),
			ID(XDLH_RETRYRAM_ADDR),
			ID(XDLH_RETRYRAM_DATA),
			ID(XDLH_RETRYRAM_EN),
			ID(XDLH_RETRYRAM_WE),
			ID(CFG_PF_TPH_ST_MODE),
			ID(CFG_TPH_REQ_EN),
			ID(CFG_VF_TPH_REQ_EN),
			ID(CFG_VF_TPH_ST_MODE),
			ID(TPH_RAM_ADDR),
			ID(TPH_RAM_FUNC_NUM),
			ID(TPH_RAM_FUNC_ACTIVE),
			ID(TPH_RAM_WR_BYTE_EN),
			ID(TPH_RAM_WR_DATA),
			ID(TPH_RAM_WR_EN)},
		       false);
	ct->setup_type(ID(GTP_PPLL),
		       {ID(CLKIN1),	 ID(CLKIN2),	  ID(CLKFB),	     ID(CLKIN_SEL),   ID(CLKOUT0_SYN), ID(CLKOUT1_SYN), ID(CLKOUT2_SYN),
			ID(CLKOUT3_SYN), ID(CLKOUT4_SYN), ID(CLKOUTPHY_SYN), ID(CLKOUTF_SYN), ID(PLL_PWD),     ID(RST),		ID(APB_CLK),
			ID(APB_RST_N),	 ID(APB_ADDR),	  ID(APB_SEL),	     ID(APB_EN),      ID(APB_WRITE),   ID(APB_WDATA)},
		       {ID(CLKOUT0), ID(CLKOUT0N), ID(CLKOUT1), ID(CLKOUT1N), ID(CLKOUT2), ID(CLKOUT2N), ID(CLKOUT3), ID(CLKOUT3N), ID(CLKOUT4),
			ID(CLKOUTPHY), ID(CLKOUTPHYN), ID(CLKOUTF), ID(CLKOUTFN), ID(LOCK), ID(APB_RDATA), ID(APB_READY)},
		       false);
	ct->setup_type(ID(GTP_PPLL_DFT),
		       {ID(CLKIN1),	 ID(CLKIN2),	  ID(CLKFB),	     ID(CLKIN_SEL),   ID(CLKOUT0_SYN), ID(CLKOUT1_SYN), ID(CLKOUT2_SYN),
			ID(CLKOUT3_SYN), ID(CLKOUT4_SYN), ID(CLKOUTPHY_SYN), ID(CLKOUTF_SYN), ID(PLL_PWD),     ID(RST),		ID(APB_CLK),
			ID(APB_RST_N),	 ID(APB_ADDR),	  ID(APB_SEL),	     ID(APB_EN),      ID(APB_WRITE),   ID(APB_WDATA)},
		       {ID(PFDTOP_CLK_TEST), ID(CLKOUT0), ID(CLKOUT0N), ID(CLKOUT1), ID(CLKOUT1N), ID(CLKOUT2), ID(CLKOUT2N), ID(CLKOUT3),
			ID(CLKOUT3N), ID(CLKOUT4), ID(CLKOUTPHY), ID(CLKOUTPHYN), ID(CLKOUTF), ID(CLKOUTFN), ID(LOCK), ID(APB_RDATA), ID(APB_READY)},
		       false);
	ct->setup_type(ID(GTP_RAM32X1DP), {ID(DI), ID(RADDR), ID(WADDR), ID(WCLK), ID(WE)}, {ID(DO)}, false);
	ct->setup_type(ID(GTP_RAM32X1SP), {ID(DI), ID(ADDR), ID(WCLK), ID(WE)}, {ID(DO)}, false);
	ct->setup_type(ID(GTP_RAM32X2DP), {ID(DI), ID(RADDR), ID(WADDR), ID(WCLK), ID(WE)}, {ID(DO)}, false);
	ct->setup_type(ID(GTP_RAM32X2SP), {ID(DI), ID(ADDR), ID(WCLK), ID(WE)}, {ID(DO)}, false);
	ct->setup_type(ID(GTP_RAM32X2X4), {ID(DI0), ID(DI1), ID(DI2), ID(DI3), ID(ADDR0), ID(ADDR1), ID(ADDR2), ID(ADDR3), ID(WCLK), ID(WE)},
		       {ID(DO0), ID(DO1), ID(DO2), ID(DO3)}, false);
	ct->setup_type(ID(GTP_RAM64X1DP), {ID(DI), ID(RADDR), ID(WADDR), ID(WCLK), ID(WE)}, {ID(DO)}, false);
	ct->setup_type(ID(GTP_RAM64X1SP), {ID(DI), ID(ADDR), ID(WCLK), ID(WE)}, {ID(DO)}, false);
	ct->setup_type(ID(GTP_RAM64X1X4), {ID(DI0), ID(DI1), ID(DI2), ID(DI3), ID(ADDR0), ID(ADDR1), ID(ADDR2), ID(ADDR3), ID(WCLK), ID(WE)},
		       {ID(DO0), ID(DO1), ID(DO2), ID(DO3)}, false);
	ct->setup_type(ID(GTP_RAM128X1DP), {ID(DI), ID(RADDR), ID(WADDR), ID(WCLK), ID(WE)}, {ID(DO)}, false);
	ct->setup_type(ID(GTP_RAM128X1SP), {ID(DI), ID(ADDR), ID(WCLK), ID(WE)}, {ID(DO)}, false);
	ct->setup_type(ID(GTP_RAM256X1SP), {ID(DI), ID(ADDR), ID(WCLK), ID(WE)}, {ID(DO)}, false);
	ct->setup_type(ID(GTP_RES_CAL_E1), {ID(PCODE_IN), ID(NCODE_IN), ID(SAMPLE_IN), ID(EN), ID(CODE_SEL), ID(RST_N)},
		       {ID(PCODE_OUT), ID(NCODE_OUT), ID(CAL_DONE)}, false);
	ct->setup_type(ID(GTP_RES_CAL_E1_DFT), {ID(PCODE_IN), ID(NCODE_IN), ID(SAMPLE_IN), ID(EN), ID(CODE_SEL), ID(RST_N)},
		       {ID(PCODE_OUT), ID(NCODE_OUT), ID(CAL_DONE)}, false);
	ct->setup_type(ID(GTP_RES_CAL_E2), {ID(PCODE_IN), ID(NCODE_IN), ID(SAMPLE_IN), ID(EN), ID(CODE_SEL), ID(RST_N)},
		       {ID(PCODE_OUT), ID(NCODE_OUT), ID(CAL_DONE)}, false);
	ct->setup_type(ID(GTP_ROM32X1), {ID(I0), ID(I1), ID(I2), ID(I3), ID(I4)}, {ID(Z)}, false);
	ct->setup_type(ID(GTP_ROM32X2), {ID(I0), ID(I1), ID(I2), ID(I3), ID(I4)}, {ID(Z)}, false);
	ct->setup_type(ID(GTP_ROM64X1), {ID(I0), ID(I1), ID(I2), ID(I3), ID(I4), ID(I5)}, {ID(Z)}, false);
	ct->setup_type(ID(GTP_ROM128X1), {ID(I0), ID(I1), ID(I2), ID(I3), ID(I4), ID(I5), ID(I6)}, {ID(Z)}, false);
	ct->setup_type(ID(GTP_ROM256X1), {ID(I0), ID(I1), ID(I2), ID(I3), ID(I4), ID(I5), ID(I6), ID(I7)}, {ID(Z)}, false);
	ct->setup_type(
	  ID(GTP_SCANCHAIN_E1), {ID(TCK), ID(TDI), ID(TMS), ID(TDO_USER)},
	  {ID(TDO), ID(CAPDR), ID(SHFTDR), ID(UPDR), ID(JCLK), ID(RST), ID(JRTI), ID(FLG_USER), ID(TCK_USER), ID(TDI_USER), ID(TMS_USER)}, false);
	ct->setup_type(ID(GTP_START_E1), {ID(CLK), ID(GOE), ID(GRS_N), ID(GWE)}, {ID(WAKEUP_OVER)}, false);
	ct->setup_type(ID(GTP_UDID), {ID(DI), ID(LOAD), ID(SE), ID(CLK)}, {ID(DO)}, false);
	ct->setup_type(ID(GTP_ZERO), {}, {ID(Z)}, false);
	ct->setup_type(ID(GTP_ZEROHOLDDELAY), {ID(DI)}, {ID(DO)}, false);
}

// generate all cut rooted on cell output
// save it to cell2cuts
bool GenerateCuts(Cell *cell)
{
	if (!IsCombinationalGate(cell)) {
		return false;
	}
	dict<pool<SigBit>, pool<Cell *>> &cuts = cell2cuts[cell];
	pool<SigBit> default_cut;
	GetCellInputsSet(cell, default_cut);
	pool<Cell *> cone;
	cone.insert(cell);
	cuts[default_cut] = cone;
	vector<pool<SigBit>> tmp_cuts;
	tmp_cuts.push_back(default_cut);
	for (size_t i = 0; i < MAX_CUT_SIZE_PRE_CELL && i < tmp_cuts.size(); ++i) {
		pool<SigBit> cut = tmp_cuts[i];
		if (cut.size() > LUT_SIZE) {
			continue;
		}
		for (SigBit &cur_bit : cut) {
			pool<SigBit> ncut = cut;
			Cell *drv = bit2driver.count(cur_bit) ? bit2driver[cur_bit] : nullptr;
			if (!drv || !IsCombinationalGate(drv)) {
				continue;
			}
			ncut.erase(cur_bit);
			pool<SigBit> tinputs;
			GetCellInputsSet(drv, tinputs);
			ncut.insert(tinputs.begin(), tinputs.end());

			if (ncut.size() > LUT_SIZE) {
				continue;
			}
			if (cuts.count(ncut)) {
				continue;
			}
			cuts.insert(ncut);
			tmp_cuts.push_back(ncut);
			pool<Cell *> ncone = cuts[cut];
			ncone.insert(drv);
			cuts[ncut] = ncone;
		}
	}
	log_debug("cell %s has %ld cuts\n", cell->name.c_str(), tmp_cuts.size());
	return true;
}

bool GenerateCuts(Module *module)
{
	pool<Cell *> cells;
	GetGates(module, cells); // generate cuts for all combinational gates
	for (Cell *cell : cells) {
		GenerateCuts(cell);
	}
	return true;
}

bool GetBestCut(Cell *cell, pool<SigBit> &cut_selected)
{
	cut_selected.clear();
	// depth-oriented cut selection
	const dict<pool<SigBit>, pool<Cell *>> &cuts = cell2cuts[cell];
	if (cuts.size() == 1) {
		cut_selected = cuts.begin()->first;
		return cut_selected.size() > 0;
	}
	float selected_depth = 1e9;
	float selected_af = 1e9;
	log_assert(cuts.size() > 0);

	float min_af = 1e9;
	pool<SigBit> min_af_cut;
	for (auto cutpair : cuts) {
		pool<SigBit> cur_cut = cutpair.first;
		float cur_depth = -1e9;
		float cur_af = 0;
		for (auto &bit : cur_cut) {
			cur_depth = max(bit2depth[bit], cur_depth);
			cur_af += bit2af[bit];
		}

		if (cur_af < min_af) {
			min_af = cur_af;
			min_af_cut = cur_cut;
		}

		if (0 == cur_interation) {
			if (abs(cur_depth - selected_depth) < 0.01 && cur_af < selected_af) {
				cut_selected = cur_cut;
				selected_depth = cur_depth;
				selected_af = cur_af;
			} else if (cur_depth < selected_depth) {
				cut_selected = cur_cut;
				selected_depth = cur_depth;
				selected_af = cur_af;
			}
		} else {
			SigBit outbit = GetCellOutput(cell);
			if (cur_depth > cell2OptDepth[cell] - bit2height[outbit]) {
				continue;
			}
			if (cur_af < selected_af) {
				cut_selected = cur_cut;
				selected_depth = cur_depth;
				selected_af = cur_af;
			}
		}
	}

	if (cut_selected.size() == 0) {
		// choose noting
		log_warning("cell %s not selected any cut at interation %ld\n", cell->name.c_str(), cur_interation);
		cut_selected = min_af_cut;
	}
	return cut_selected.size() > 0;
}

float GetEstimatedFanout(SigBit bit)
{
	float alpha = 2.5;
	size_t fanout_est = bit2fanout_est[bit];
	size_t fanout = bit2reader[bit].size();
	float est = (fanout_est + alpha * fanout) / (1 + alpha);
	return est;
}

bool UpdateCutDepthAf(const pool<SigBit> &cut_selected, Cell *cell, SigBit outbit)
{
	float depth = 0;
	float af = 0;
	for (auto &bit : cut_selected) {
		af += bit2af[bit];
		depth = max(depth, bit2depth[bit]);
	}

	float fanout_est = GetEstimatedFanout(outbit);
	bit2depth[outbit] = depth + 1;
	bit2af[outbit] = af / fanout_est;

	return true;
}

bool TraverseFWD(Module *module, const pool<SigBit> &prime_inputs, dict<SigBit, pool<SigBit>> &bit2cut)
{
	for (SigBit pi : prime_inputs) {
		bit2depth[pi] = 0.0;
		bit2af[pi] = 0.0;
	}

	vector<Cell *> gates;
	GetTopoSortedGates(module, gates);
	for (size_t i = 0; i < gates.size(); i++) {
		pool<SigBit> cut_selected;
		if (!GetBestCut(gates[i], cut_selected)) {
			log_error(" not selected cut %s\n", gates[i]->name.c_str());
		}
		SigBit outbit = GetCellOutput(gates[i]);
		bit2cut[outbit] = cut_selected;
		UpdateCutDepthAf(cut_selected, gates[i], outbit);
	}
	return true;
}
bool TraverseBWD(Module *module, const pool<SigBit> &prime_outputs, dict<SigBit, pool<SigBit>> &bit2cut)
{
	dict<SigBit, pool<SigBit>> map_result;
	dict<SigBit, size_t> bit2fanout_bwd;
	pool<Cell *> s_for_check;
	for (SigBit po : prime_outputs) {
		bit2height[po] = 0.0;
		if (bit2driver.count(po)) {
			Cell *drv_cell = bit2driver[po];
			if (IsCombinationalGate(drv_cell)) {
				s_for_check.insert(drv_cell);
			}
		}
		bit2fanout_bwd[po] = 0;
	}

	vector<Cell *> gates;
	GetTopoSortedGates(module, gates);

	for (auto it = gates.rbegin(); it != gates.rend(); ++it) {
		Cell *cell = *it;
		log_assert(IsCombinationalGate(cell));
		SigBit outbit = GetCellOutput(cell);
		if (!s_for_check.count(cell)) {
			continue;
		}
		float cone_h = bit2height[outbit];
		pool<SigBit> cut_selected = bit2cut[outbit];
		if (!map_result.count(outbit)) {
			map_result[outbit] = cut_selected;
		} else {
			log_error("found cycle at %s\n", log_signal(outbit));
			continue;
		}
		pool<Cell *> cone = cell2cuts[cell][cut_selected];
		for (Cell *cell : cone) {
			SigBit tmpbit = GetCellOutput(cell);
			bit2height[tmpbit] = max(bit2height[tmpbit], cone_h);
		}
		for (auto &bit : cut_selected) {
			bit2fanout_bwd[bit]++;
			bit2height[bit] = max(bit2height[bit], cone_h + 1);
			if (bit2driver.count(bit)) {
				Cell *drv_cell = bit2driver[bit];
				if (IsCombinationalGate(drv_cell)) {
					s_for_check.insert(drv_cell);
				}
			}
		}
	}

	for (auto &pp : bit2fanout_est) {
		SigBit bit = pp.first;
		size_t fo = pp.second;
		if (bit2fanout_bwd.count(bit)) {
			bit2fanout_est[bit] = bit2fanout_bwd[bit];
		} else {
			bit2fanout_est[bit] = 0; // this bit is not in any cut, maybe is a internal bit
		}
		for (auto reader : bit2reader[bit]) {
			bit2fanout_est[bit] += IsGTP(reader);
		}
	}
	bit2cut.clear();
	for (auto &p : map_result) {
		bit2cut[p.first] = p.second;
	}
	return true;
}
// evaluate SigBit out is true or false base on inputs(bit_map)
State StateEval(dict<SigBit, State> &bit_map, SigBit out)
{
	if (bit_map.count(out)) {
		return bit_map[out];
	}
	Cell *cell = bit2driver[out];
	if (!cell) {
		return State::Sx;
	}

	vector<SigBit> bits = cell2bits[cell];
	if (IsAND(cell)) {
		SigBit tout = bits[0];
		for (size_t i = 1; i < bits.size(); i++) {
			State tmp = StateEval(bit_map, bits[i]);
			if (tmp == State::S0) {
				bit_map[tout] = State::S0;
				return State::S0;
			} else if (tmp == State::S1) {
				continue;
			} else {
				log_error("Cannot evaluate %s \n", log_signal(tout));
				bit_map[tout] = State::Sx;
				return State::Sx;
			}
		}
		return State::S1;
	} else if (IsOR(cell)) {
		SigBit tout = bits[0];
		for (size_t i = 1; i < bits.size(); i++) {
			State tmp = StateEval(bit_map, bits[i]);
			if (tmp == State::S1) {
				bit_map[tout] = State::S1;
				return State::S1;
			} else if (tmp == State::S0) {
				continue;
			} else {
				log_error("Cannot evaluate %s \n", log_signal(tout));
				bit_map[tout] = State::Sx;
				return State::Sx;
			}
		}
		return State::S0;
	} else if (IsNOT(cell)) {
		SigBit tout = bits[0];
		State tmp = StateEval(bit_map, bits[1]);
		if (tmp == State::S0) {
			bit_map[tout] = State::S1;
			return State::S1;
		} else if (tmp == State::S1) {
			bit_map[tout] = State::S0;
			return State::S0;
		} else {
			log_error("Cannot evaluate %s \n", log_signal(tout));
		}
	} else if (IsMUX(cell)) {
		SigBit tout = bits[0];
		SigSpec sig = cell->getPort(ID(S));
		sig = sigmap(sig);
		log_assert(sig.size());
		SigBit sel_bit = sig[0];
		sig = cell->getPort(ID(A));
		sig = sigmap(sig);
		log_assert(sig.size());
		SigBit i0_bit = sig[0];
		sig = cell->getPort(ID(B));
		sig = sigmap(sig);
		log_assert(sig.size());
		SigBit i1_bit = sig[0];
		State sel_state = StateEval(bit_map, sel_bit);
		if (sel_state == State::S0) {
			State tmp = StateEval(bit_map, i0_bit);
			bit_map[tout] = tmp;
			return tmp;
		} else if (sel_state == State::S1) {
			State tmp = StateEval(bit_map, i1_bit);
			bit_map[tout] = tmp;
			return tmp;
		}
	} else if (IsXOR(cell)) {
		SigBit tout = bits[0];
		State tmp0 = StateEval(bit_map, bits[1]);
		State tmp1 = StateEval(bit_map, bits[2]);
		if (tmp0 == State::Sx || tmp1 == State::Sx) {
			log_error("Cannot evaluate %s \n", log_signal(tout));
			bit_map[tout] = State::Sx;
			return State::Sx;
		} else if (tmp0 == tmp1) {
			bit_map[tout] = State::S0;
			return State::S0;
		} else {
			bit_map[tout] = State::S1;
			return State::S1;
		}
	} else {
		log_error("unhandled cell %s \n", cell->type.c_str());
	}
	return State::Sx;
}

vector<bool> GetCutInit(const vector<SigBit> &cut, SigBit out)
{
	log_assert(cut.size() <= LUT_SIZE && cut.size() >= 1);
	size_t bits_num = size_t(1 << cut.size());
	vector<bool> cut_init(bits_num, false);
	for (size_t i = 0; i < bits_num; i++) {
		dict<SigBit, State> bit_map;
		for (size_t n = 0; n < cut.size(); n++)
			bit_map[cut[n]] = ((i >> n) & 1) ? State::S1 : State::S0;

		State value = StateEval(bit_map, out);
		if (value == State::Sx) {
			log_error("Cannot evaluate %s \n", log_signal(out));
		}
		cut_init[i] = value == State::S1;
	}
	return cut_init;
}

RTLIL::Cell *addLut(Module *module, const pool<SigBit> &cut, const RTLIL::SigBit &sig_z)
{
	log_assert(cut.size() <= LUT_SIZE && cut.size() >= 1);
	vector<SigBit> vcut;
	for (auto bit : cut) {
		vcut.push_back(bit);
	}
	vector<bool> cut_init_bools = GetCutInit(vcut, sig_z);
	Cell *drv = bit2driver[sig_z];
	log_assert(drv);
	IdString name = drv->name;
	string new_name = string(name.c_str()) + "_lut";
	Cell *cell = nullptr;
	if (using_internel_lut_type) {
		// instantiate $lut, need call techmap pass
		cell = module->addCell(IdString(new_name), ID($lut));
		cell->parameters[ID::WIDTH] = RTLIL::Const(vcut.size());
		cell->parameters[ID::LUT] = RTLIL::Const(cut_init_bools);

		cell->setPort(ID(A), vcut);
		cell->setPort(ID(Y), sig_z);
		cell->set_src_attribute(drv->get_src_attribute());
	} else {
		IdString types[] = {ID(GTP_LUT1), ID(GTP_LUT2), ID(GTP_LUT3), ID(GTP_LUT4), ID(GTP_LUT5), ID(GTP_LUT6)};
		cell = module->addCell(RTLIL::IdString(new_name), types[vcut.size() - 1]);
		cell->parameters[ID::INIT] = RTLIL::Const(cut_init_bools);
		for (size_t i = 0; i < vcut.size(); ++i) {
			string pin_name = "\\I" + to_string(i);
			cell->setPort(RTLIL::IdString(pin_name), vcut[i]);
		}
		cell->setPort(ID(Z), sig_z);
		cell->set_src_attribute(drv->get_src_attribute());
	}
	return cell;
}
// map selected cut in bit2cut to GTP_LUT
bool ConeToLUTs(Module *module, dict<SigBit, pool<SigBit>> &bit2cut)
{
	static size_t vf_count = 0;
	for (auto &p : bit2cut) {
		vf_count++;
		SigBit out = p.first;
		pool<SigBit> cut = p.second;
		Cell *lut = addLut(module, cut, out);
		log_assert(lut);
	}
	for (auto c : module->cells_) {
		if (IsCombinationalGate(c.second)) {
			module->remove(c.second);
		}
	}
	return true;
}

bool CheckCellWidth(Module *module)
{
	log_debug("check cell width in module\n");
	for (Cell *cell : module->cells()) {
		if (IsMUX(cell)) {

		} else if (IsAND(cell) || IsOR(cell) || IsXOR(cell)) {

		} else if (IsNOT(cell) || IsGTP(cell)) {

		} else if (cell) {
			log_error("find unsupported cell %s (%s).\n", cell->name.c_str(), cell->type.c_str());
		}
	}

	log_debug("Init driver/reader dict\n");
	for (auto &cell_iter : module->cells_) {
		Cell *cell = cell_iter.second;
		if (!yosys_celltypes.cell_known(cell->type)) {
			log_warning("cell %s (%s) is not a know type.\n", cell->name.c_str(), cell->type.c_str());
			continue;
		}

		vector<SigBit> all_bits;
		vector<SigBit> input_bits;
		for (auto &conn : cell->connections()) {
			IdString portname = conn.first;
			// (use sigmap to get a uniqe signal name)
			RTLIL::SigSpec sig = sigmap(conn.second);
			if (yosys_celltypes.cell_output(cell->type, portname)) {
				if (sig.size() == 0) {
					continue;
				}
				if (IsCombinationalCell(cell) && !sig.is_bit()) {
					log_error("cell %s %s  sig %s is not a bit=%d\n", cell->name.c_str(), cell->type.c_str(),
						  sig.as_string().c_str(), sig.size());
				}
				for (int i = 0; i < sig.size(); i++) {
					bit2driver[sig[i]] = cell;
					all_bits.push_back(sig[i]);
				}
			} else if (yosys_celltypes.cell_input(cell->type, portname)) {
				for (int i = 0; i < sig.size(); i++) {
					bit2reader[sig[i]].push_back(cell);
					input_bits.push_back(sig[i]);
				}
			}
		}
		if (all_bits.size() < 1 && cell->type != ID(GTP_GRS)) {
			log_warning(" cell %s(%s) have not any output.\n", cell->name.c_str(), cell->type.c_str());
		}
		for (auto bit : input_bits) {
			all_bits.push_back(bit);
		}
		cell2bits[cell] = all_bits;
	}
	return true;
}

struct MapperPass : public Pass {
	MapperPass() : Pass("mapper", "synthesis for Pango FPGA. Mapper main function.") {}
	void help() override
	{
		//   |---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|
		log("\n");
		log("    mapper [options] [selection]\n");
	}
	bool write_out_black_list;
	void clear_flags() override { write_out_black_list = false; }
	void execute(std::vector<std::string> args, RTLIL::Design *design) override
	{
		log_header(design, "Start MapperPass\n");

		size_t argidx;
		for (argidx = 1; argidx < args.size(); argidx++) {
			if (args[argidx] == "-ilut") {
				using_internel_lut_type = true;
				continue;
			}
			if (args[argidx] == "-interation" && argidx + 1 < args.size()) {
				MAX_INTERATIONS = max(atoi(args[++argidx].c_str()), 3);
				continue;
			}
			break;
		}
		extra_args(args, argidx, design);

		Module *module = design->top_module();
		if (module == nullptr)
			log_cmd_error("No top module found.\n");

		log_header(design, "Continuing MapperPass pass.\n");
		MapperInit(module);
		MapperMain(module);
		log_pop();
	}
} MapperPass;

struct SynthPangoPass : public ScriptPass {
	SynthPangoPass() : ScriptPass("synth_pango", "synthesis script pass for Pango FPGA. Map gate to GTP_LUT.") {}
	void help() override
	{
		//   |---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|
		log("\n");
		log("    synth_pango [options] [selection]\n");
	}

	string input_verilog_file;
	string output_verilog_file;
	string top_module_name;
	void clear_flags() override
	{
		using_internel_lut_type = false;
		output_verilog_file = "";
		top_module_name = "";
	}
	void execute(std::vector<std::string> args, RTLIL::Design *design) override
	{
		string run_from, run_to;
		clear_flags();

		size_t argidx;
		for (argidx = 1; argidx < args.size(); argidx++) {
			if (args[argidx] == "-top" && argidx + 1 < args.size()) {
				top_module_name = args[++argidx];
				continue;
			}
			if (args[argidx] == "-input" && argidx + 1 < args.size()) {
				input_verilog_file = args[++argidx];
				continue;
			}
			if (args[argidx] == "-interation" && argidx + 1 < args.size()) {
				MAX_INTERATIONS = max(atoi(args[++argidx].c_str()), 3);
				continue;
			}
			if (args[argidx] == "-run" && argidx + 1 < args.size()) {
				size_t pos = args[argidx + 1].find(':');
				if (pos == std::string::npos)
					break;
				run_from = args[++argidx].substr(0, pos);
				run_to = args[argidx].substr(pos + 1);
				continue;
			}
			break;
		}

		extra_args(args, argidx, design);
		if (!design->full_selection())
			log_cmd_error("This command only operates on fully selected designs!\n");

		log_header(design, "Start synth_pango\n");
		log_push();

		run_script(design, run_from, run_to);

		log_pop();
	}
	void script() override
	{
		if (check_label("begin")) {
#if defined(_WIN32)
			run("read_verilog -lib ./techlibs/pango/pango_lib.v");
#else
			run("read_verilog -lib +/pango/pango_lib.v");
#endif
			run(stringf("read_verilog -icells %s", input_verilog_file.c_str()));
			if (top_module_name.size() > 0) {
				run(stringf("hierarchy -check -top %s", top_module_name.c_str()));
			}
		}

		Module *module = active_design->top_module();
		if (module == nullptr)
			log_cmd_error("No top module found.\n");
		if (top_module_name.size() == 0) {
			top_module_name = module->name.c_str();
		}

		if (check_label("pango")) {
			run(stringf("hierarchy -check -top %s;;", top_module_name.c_str()));
			MapperInit(module);
			MapperMain(module);
		}
		if (check_label("check")) {
			run("check -mapped");
		}
		if (check_label("verilog")) {
			run(stringf("write_verilog -noexpr -noattr %s_syn.v", top_module_name.c_str() + 1));
		}
		if (check_label("score")) {
			run(stringf("score -before %s -after %s_syn.v", input_verilog_file.c_str(), top_module_name.c_str() + 1));
		}
	}
} SynthPangoPass;
PRIVATE_NAMESPACE_END
