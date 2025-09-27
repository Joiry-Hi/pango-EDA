#include "kernel/celltypes.h"
#include "kernel/consteval.h"
#include "kernel/modtools.h"
#include "kernel/rtlil.h"
#include "kernel/sigtools.h"
#include "kernel/yosys.h"
#include <algorithm>
#include <chrono>
#include <fstream>
#include <queue>
#include <ranges>
#include <string.h>
#include <vector>

USING_YOSYS_NAMESPACE
using namespace std;
PRIVATE_NAMESPACE_BEGIN

// LutInfo 结构体保持不变
struct LutInfo {
	RTLIL::Cell *cell_ptr = nullptr;
	int size = 0;
	// 使用 map 来存储有序的、带端口名的输入
	map<string, SigBit> ordered_inputs;
	SigBit output;
	RTLIL::Const init_val;
	bool is_merged = false;
};

// 用于存储每个LUT的拓扑排序信息
struct TopoInfo {
	int topo_id = -1;		       // 拓扑排序后的唯一编号
	int level = -1;			       // 在拓扑结构中的层级/深度
	vector<SigBit> ordered_unified_inputs; // 【关键】用于对比的、统一排序的输入信号
};

// 用于方便地通过Cell指针或SigBit查找信息
dict<RTLIL::Cell *, TopoInfo> cell_to_topo_info;
dict<RTLIL::SigBit, int> output_sig_to_topo_id; // 输出信号跟随其驱动LUT的编号

#pragma region tool_funcs
// =================================================================
// 新增的辅助函数
// =================================================================
// 注意：我们需要稍微修改一下打印函数，让它可以接受一个文件流作为参数

// 最终、最可靠的格式化函数

string format_init_hex(const RTLIL::Const &init_val)
{
	int width = GetSize(init_val);
	if (width == 0) {
		return "0";
	}

	// 创建一个非const的拷贝
	RTLIL::Const val_copy = init_val;

	string bin_str;
	bin_str.reserve(width);

	// 从高位到低位构建二进制字符串
	for (int i = width - 1; i >= 0; i--) {
		// --- 核心修正：使用 val_copy.bits().at(i) ---
		// 先调用 bits() 获得 vector 的引用，然后用 .at(i) 访问
		if (val_copy.bits().at(i) == RTLIL::S1) {
			bin_str += '1';
		} else if (val_copy.bits().at(i) == RTLIL::S0) {
			bin_str += '0';
		} else {
			bin_str += 'x';
		}
	}

	// ... (后续的二进制到十六进制转换逻辑保持不变) ...
	while (bin_str.length() % 4 != 0) {
		bin_str.insert(0, "0");
	}

	string hex_str;
	const char hex_chars[] = "0123456789abcdef";

	for (size_t i = 0; i < bin_str.length(); i += 4) {
		int nibble = 0;
		string group = bin_str.substr(i, 4);
		if (group[0] == '1')
			nibble += 8;
		if (group[1] == '1')
			nibble += 4;
		if (group[2] == '1')
			nibble += 2;
		if (group[3] == '1')
			nibble += 1;
		hex_str += hex_chars[nibble];
	}

	// 1. 检查十六进制字符串长度是否为16
	if (hex_str.length() == 16) {
		// 2. 在索引位置8（即第9个字符的位置，从0开始计数）插入下划线
		hex_str.insert(8, "_");
	}

	if (hex_str.empty()) {
		return "0";
	}

	return hex_str;
}

// 修改后的打印函数
void print_lut_info_to_stream(ostream &f, const LutInfo &info)
{
	f << "  - Cell: " << log_id(info.cell_ptr->name) << " (Type: " << log_id(info.cell_ptr->type) << ", Size: " << info.size << ")\n";
	f << "    Output: " << log_signal(info.output) << "\n";

	// --- 修改在这里 ---
	// 遍历有序字典，打印出端口名和对应的信号
	f << "    Inputs:\n";
	for (const auto &pair : info.ordered_inputs) {
		f << "      ." << pair.first << ": " << log_signal(pair.second) << "\n";
	}

	string init_hex_str = format_init_hex(info.init_val);
	f << "    INIT: " << GetSize(info.init_val) << "'h" << init_hex_str << "\n";
	f << "    INIT: " << GetSize(info.init_val) << "'b" << info.init_val.as_string() << "\n\n";
}

void dump_luts_to_file(const string &filename, const vector<LutInfo> &luts)
{
	ofstream f(filename);
	if (!f.is_open()) {
		log_error("Could not open file '%s' for writing.\n", filename.c_str());
		return;
	}

	f << "--- Dump of all collected LUTs (" << luts.size() << " total) ---\n\n";
	for (const auto &lut : luts) {
		print_lut_info_to_stream(f, lut);
	}
	f << "--- End of LUT dump ---\n";
	log("Successfully dumped LUT info to '%s'.\n", filename.c_str());
}
#pragma endregion tool_funcs

// =================================================================
// 最终、最可靠的 INIT 计算函数 (已修复逻辑漏洞)
// =================================================================
RTLIL::Const calculate_new_init(const LutInfo &lut_a, const LutInfo &lut_b, const vector<SigBit> &new_inputs_vec, const SigBit &sel_bit,
				SigBit &z_out_sig, SigBit &z5_out_sig)
{
	// --- 准备工作 (已修复) ---
	// 辅助函数：检查一个LUT是否包含某个输入信号
	auto lut_has_input = [](const LutInfo &lut, const SigBit &input_sig) {
		for (const auto &pair : lut.ordered_inputs) {
			if (pair.second == input_sig)
				return true;
		}
		return false;
	};

	// 正确的逻辑：不包含sel_bit的LUT用于Z5 (sel=0)，包含sel_bit的用于Z (sel=1)
	const LutInfo &lut_for_z5 = lut_has_input(lut_a, sel_bit) ? lut_b : lut_a;
	const LutInfo &lut_for_z_sel1 = lut_has_input(lut_a, sel_bit) ? lut_a : lut_b;

	z5_out_sig = lut_for_z5.output;
	z_out_sig = lut_for_z_sel1.output;

	vector<SigBit> shared_inputs;
	for (size_t i = 0; i < 5; ++i)
		shared_inputs.push_back(new_inputs_vec[i]);

	RTLIL::Const init_a_copy = lut_for_z5.init_val;
	RTLIL::Const init_b_copy = lut_for_z_sel1.init_val;

	vector<RTLIL::State> z5_bits, z_bits;
	z5_bits.reserve(32);
	z_bits.reserve(32);

	// 1. 计算 Z5 (sel=0) 的 32-bit 真值表
	for (int i = 0; i < 32; ++i) {
		size_t addr_a = 0;
		int bit_pos = 1;
		for (const auto &pair : lut_for_z5.ordered_inputs) {
			auto it = find(shared_inputs.begin(), shared_inputs.end(), pair.second);
			if (it != shared_inputs.end()) {
				int shared_idx = distance(shared_inputs.begin(), it);
				if ((i >> shared_idx) & 1)
					addr_a += bit_pos;
			}
			// 当计算Z5的逻辑时，sel_bit的值被认为是0，所以不需要做任何事
			bit_pos <<= 1;
		}

		if (addr_a < (size_t)GetSize(init_a_copy)) {
			z5_bits.push_back(init_a_copy.bits().at(addr_a));
		} else {
			z5_bits.push_back(RTLIL::S0);
		}
	}

	// 2. 计算 Z (sel=1) 的 32-bit 真值表
	for (int i = 0; i < 32; ++i) {
		size_t addr_b = 0;
		int bit_pos = 1;
		for (const auto &pair : lut_for_z_sel1.ordered_inputs) {
			auto it = find(shared_inputs.begin(), shared_inputs.end(), pair.second);
			if (it != shared_inputs.end()) {
				int shared_idx = distance(shared_inputs.begin(), it);
				if ((i >> shared_idx) & 1)
					addr_b += bit_pos;
			} else if (pair.second == sel_bit) {
				// 当计算Z的逻辑时，sel_bit的值被认为是1
				addr_b += bit_pos;
			}
			bit_pos <<= 1;
		}

		if (addr_b < (size_t)GetSize(init_b_copy)) {
			z_bits.push_back(init_b_copy.bits().at(addr_b));
		} else {
			z_bits.push_back(RTLIL::S0);
		}
	}

	// --- 写入阶段 ---
	vector<RTLIL::State> final_init_bits = z5_bits;
	final_init_bits.insert(final_init_bits.end(), z_bits.begin(), z_bits.end());

	return RTLIL::Const(final_init_bits);
}

// =================================================================
// 步骤 1: 提取所有GTP_LUT的信息
// =================================================================
void CollectLuts(Module *module, SigMap &sigmap, vector<LutInfo> &luts)
{
	luts.clear();
	for (Cell *cell : module->cells()) {
		const char *type_str = cell->type.c_str();
		if (strncmp(type_str, "\\GTP_LUT", 8) == 0 && strlen(type_str) == 9) {
			LutInfo info;
			info.cell_ptr = cell;
			info.size = type_str[8] - '0';

			// ---  修改在这里 ---
			// 按照端口名 I0, I1, ... 顺序提取输入
			for (int i = 0; i < info.size; ++i) {
				string port_name = "I" + to_string(i);
				IdString port_id = IdString("\\" + port_name); // Yosys内部端口名通常带'\'

				if (cell->hasPort(port_id)) {
					// 将 (端口名, 信号) 存入有序字典
					info.ordered_inputs[port_name] = sigmap(cell->getPort(port_id));
				}
			}

			info.output = sigmap(cell->getPort(ID(Z)));
			info.init_val = cell->getParam(ID(INIT));
			luts.push_back(info);
		}
	}
}

// =================================================================
// 步骤 1.5: 对所有LUT进行拓扑排序和编号
// =================================================================
void NumberAndSortLuts(Module *module, const vector<LutInfo> &luts, SigMap &sigmap)
{
	log("Numbering and sorting LUTs based on topology...\n");

	// --- 准备工作：建立图的邻接关系和入度表 ---
	dict<RTLIL::Cell *, pool<RTLIL::Cell *>> lut_graph; // LUT -> set of downstream LUTs
	dict<RTLIL::Cell *, int> lut_in_degree;
	dict<RTLIL::SigBit, RTLIL::Cell *> output_sig_to_lut; // 方便快速查找驱动LUT

	for (const auto &lut : luts) {
		lut_in_degree[lut.cell_ptr] = 0; // 初始化入度
		output_sig_to_lut[lut.output] = lut.cell_ptr;
	}

	for (const auto &src_lut : luts) {
		// 找出 src_lut 的所有下游 reader LUTs
		pool<RTLIL::Cell *> readers = module->readers(src_lut.output);
		for (RTLIL::Cell *reader_cell : readers) {
			// 检查这个 reader 是否也是我们关心的GTP_LUT
			if (output_sig_to_lut.count(reader_cell->connections().at(ID(Z)))) {
				// 是的，这是一个下游LUT
				RTLIL::Cell *dest_lut_ptr = output_sig_to_lut.at(reader_cell->connections().at(ID(Z)));

				// 确保我们只添加一次边，并增加目标LUT的入度
				if (lut_graph[src_lut.cell_ptr].insert(dest_lut_ptr).second) {
					lut_in_degree[dest_lut_ptr]++;
				}
			}
		}
	}

	// --- 拓扑排序 (Kahn's Algorithm using BFS) ---
	queue<pair<RTLIL::Cell *, int>> q; // pair: {Cell*, level}
	for (const auto &lut : luts) {
		if (lut_in_degree[lut.cell_ptr] == 0) {
			q.push({lut.cell_ptr, 0}); // level 0
		}
	}

	int current_topo_id = 0;
	while (!q.empty()) {
		auto [current_lut_ptr, current_level] = q.front();
		q.pop();

		// 1. 分配编号和层级
		cell_to_topo_info[current_lut_ptr].topo_id = current_topo_id;
		cell_to_topo_info[current_lut_ptr].level = current_level;

		// 2. 让输出信号跟随驱动LUT的编号
		SigBit out_sig = current_lut_ptr->connections().at(ID(Z));
		output_sig_to_topo_id[out_sig] = current_topo_id;

		// 3. 【关键】为输入信号排序并存储
		vector<SigBit> inputs;
		for (const auto &pair : current_lut_ptr->connections()) {
			if (pair.first.begins_with("\\I")) {
				inputs.push_back(sigmap(pair.second));
			}
		}

		// 排序规则：首先按信号类型（模块输入/中间信号），然后按拓扑ID或线号
		std::sort(inputs.begin(), inputs.end(), [&](const SigBit &a, const SigBit &b) {
			bool a_is_intermediate = output_sig_to_topo_id.count(a);
			bool b_is_intermediate = output_sig_to_topo_id.count(b);

			if (a_is_intermediate && !b_is_intermediate)
				return false; // 中间信号排后面
			if (!a_is_intermediate && b_is_intermediate)
				return true; // 模块输入排前面

			if (a_is_intermediate && b_is_intermediate) {
				// 两者都是中间信号，按驱动LUT的拓扑ID排序
				return output_sig_to_topo_id[a] < output_sig_to_topo_id[b];
			} else {
				// 两者都是模块输入，按线号（wire index）排序
				return a.wire->name.index() < b.wire->name.index();
			}
		});
		cell_to_topo_info[current_lut_ptr].ordered_unified_inputs = inputs;

		// 4. 更新下游LUT的入度
		for (RTLIL::Cell *downstream_lut_ptr : lut_graph[current_lut_ptr]) {
			if (--lut_in_degree[downstream_lut_ptr] == 0) {
				q.push({downstream_lut_ptr, current_level + 1});
			}
		}

		current_topo_id++;
	}

	if (current_topo_id != (int)luts.size()) {
		log_warning("Combinational loop detected among LUTs. Topological numbering may be incomplete.\n");
	}
}

// =================================================================
// 步骤 2: 寻找并评估所有可合并的候选对
// =================================================================
struct MergeCandidate {
	int idx_a, idx_b;
	int score; // 分数越高越好
	pool<SigBit> union_inputs;

	bool operator<(const MergeCandidate &other) const
	{
		return score < other.score; // 用于优先队列
	}
};

void FindMergeCandidates(const vector<LutInfo> &luts, priority_queue<MergeCandidate> &candidates)
{
	for (size_t i = 0; i < luts.size(); ++i) {
		for (size_t j = i + 1; j < luts.size(); ++j) {
			const LutInfo &lut_a = luts[i];
			const LutInfo &lut_b = luts[j];

			// 合并检查
			pool<SigBit> current_union_inputs;
			for (const auto &pair : lut_a.ordered_inputs)
				current_union_inputs.insert(pair.second);
			for (const auto &pair : lut_b.ordered_inputs)
				current_union_inputs.insert(pair.second);

			int shared_inputs = (lut_a.size + lut_b.size) - current_union_inputs.size();

			if (current_union_inputs.size() <= 6 && shared_inputs <= 5) {
				if (current_union_inputs.size() <= 6) {
					// 计算分数：共享输入数 * 100 - 总输入数
					int shared_inputs = (lut_a.size + lut_b.size) - current_union_inputs.size();
					int score = shared_inputs * 100 - current_union_inputs.size();
					candidates.push({(int)i, (int)j, score, current_union_inputs});
				}
			}
		}
	}
}

// =================================================================
// 步骤 3: 执行合并操作
// =================================================================
// 用于存储合并计划的安全结构体，不包含任何实时指针
struct MergePlan {
	RTLIL::IdString new_cell_name;
	RTLIL::Const init_val;
	map<RTLIL::IdString, RTLIL::SigBit> port_connections;
	RTLIL::IdString cell_a_to_remove; // 只存储名字
	RTLIL::IdString cell_b_to_remove; // 只存储名字
};
// 新版本：此函数只负责规划，返回一个安全的计划列表
vector<MergePlan> PlanMerges(Module *module, vector<LutInfo> &luts, priority_queue<MergeCandidate> &candidates)
{
	vector<MergePlan> plans;

	while (!candidates.empty()) {
		MergeCandidate best_pair = candidates.top();
		candidates.pop();

		LutInfo &lut_a = luts[best_pair.idx_a];
		LutInfo &lut_b = luts[best_pair.idx_b];

		if (lut_a.is_merged || lut_b.is_merged)
			continue;

		lut_a.is_merged = true;
		lut_b.is_merged = true;

		// --- 核心计算逻辑 (保持不变) ---
		vector<SigBit> new_inputs_vec;
		for (const auto &sig : best_pair.union_inputs)
			new_inputs_vec.push_back(sig);
		while (new_inputs_vec.size() < 6)
			new_inputs_vec.push_back(RTLIL::S0);

		pool<SigBit> inputs_a, inputs_b;
		for (const auto &p : lut_a.ordered_inputs)
			inputs_a.insert(p.second);
		for (const auto &p : lut_b.ordered_inputs)
			inputs_b.insert(p.second);

		SigBit sel_bit = RTLIL::Sx;
		for (const auto &sig : new_inputs_vec) {
			if (sig != RTLIL::S0 && (inputs_a.count(sig) != inputs_b.count(sig))) {
				sel_bit = sig;
				break; // 找到一个独立信号作为SEL信号，但不一定需要吧？
			}
		}
		if (sel_bit == RTLIL::Sx) {
			for (const auto &sig : new_inputs_vec) {
				if (sig != RTLIL::S0) {
					sel_bit = sig;
					break;
				}
			}
		}
		auto sel_it = find(new_inputs_vec.begin(), new_inputs_vec.end(), sel_bit);
		if (sel_it != new_inputs_vec.end())
			iter_swap(sel_it, new_inputs_vec.end() - 1);

		SigBit z_out_target, z5_out_target;
		RTLIL::Const new_init = calculate_new_init(lut_a, lut_b, new_inputs_vec, sel_bit, z_out_target, z5_out_target);

		// --- 创建一个安全的计划 ---
		MergePlan plan;
		string new_name_str = string(log_id(lut_a.cell_ptr->name)) + "_" + string(log_id(lut_b.cell_ptr->name)) + "_merged";
		plan.new_cell_name = module->uniquify(RTLIL::IdString("\\" + new_name_str));
		plan.init_val = new_init;

		for (size_t k = 0; k < 6; ++k)
			plan.port_connections[IdString("\\I" + to_string(k))] = new_inputs_vec[k];
		plan.port_connections[ID(Z)] = z_out_target;
		plan.port_connections[ID(Z5)] = z5_out_target;

		plan.cell_a_to_remove = lut_a.cell_ptr->name;
		plan.cell_b_to_remove = lut_b.cell_ptr->name;

		plans.push_back(plan);
	}
	return plans;
}

// 新版本：StitcherMain 负责调用规划和执行
void StitcherMain(Module *module, const std::string &dump_filename)
{
	auto start_time = std::chrono::high_resolution_clock::now();

	log("Step 1: Scanning and collecting all GTP_LUTs...\n");
	SigMap sigmap(module);
	vector<LutInfo> all_luts;
	CollectLuts(module, sigmap, all_luts);

	// --- 新增步骤：在收集信息后，立即进行编号和排序 ---
	NumberAndSortLuts(module, all_luts, sigmap);

	if (!dump_filename.empty()) {
		dump_luts_to_file(dump_filename, all_luts);
	}

	log("Step 2: Finding best pairs to merge...\n");
	priority_queue<MergeCandidate> candidates;
	FindMergeCandidates(all_luts, candidates);
	log("Found %zu potential merge candidates.\n", candidates.size());

	// --- 阶段一: 规划 ---
	vector<MergePlan> plans = PlanMerges(module, all_luts, candidates);
	if (plans.empty()) {
		log("No valid merges found.\n");
		return;
	}
	log("Planned %zu merges.\n", plans.size());

	// --- 阶段二: 执行 (安全的"先移除，后添加"策略) ---
	pool<RTLIL::IdString> cell_names_to_remove;
	for (const auto &plan : plans) {
		cell_names_to_remove.insert(plan.cell_a_to_remove);
		cell_names_to_remove.insert(plan.cell_b_to_remove);
	}

	// 1. 先通过名字安全地移除所有旧单元
	for (auto cell_name : cell_names_to_remove) {
		if (module->cell(cell_name)) {
			module->remove(module->cell(cell_name));
		}
	}

	// 2. 然后添加所有新的 LUT6D 单元
	for (const auto &plan : plans) {
		Cell *new_lut = module->addCell(plan.new_cell_name, ID(GTP_LUT6D));
		new_lut->setParam(ID::INIT, plan.init_val);
		for (const auto &conn : plan.port_connections) {
			new_lut->setPort(conn.first, conn.second);
		}
	}

	log("Performed %zu merges successfully.\n", plans.size());

	auto end_time = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> elapsed_ms = end_time - start_time;
	log("Whole process took %.2f ms.\n", elapsed_ms.count());
}

struct StitcherPass : public Pass {
	StitcherPass() : Pass("stitcher", "Basic Task: find and stitch GTP_LUTs.") {}

	std::string dump_filename;
	void clear_flags() override { dump_filename = ""; }

	void execute(vector<string> args, RTLIL::Design *design) override
	{
		log_header(design, "Executing StitcherPass (Basic Task).\n");
		clear_flags();

		size_t argidx;
		for (argidx = 1; argidx < args.size(); argidx++) {
			if (args[argidx] == "-dump" && argidx + 1 < args.size()) {
				dump_filename = args[++argidx];
				continue;
			}
			break;
		}
		extra_args(args, argidx, design);

		Module *module = design->top_module();
		if (module == nullptr)
			log_cmd_error("No top module found.\n");

		// --- 关键修改：直接在顶层模块上执行，不再进行任何克隆或替换 ---
		log("Performing in-place LUT stitching on module: %s\n", log_id(module));
		StitcherMain(module, dump_filename);

		log("Stitching complete.\n");
	}
} StitcherPass;

PRIVATE_NAMESPACE_END