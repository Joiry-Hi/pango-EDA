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

	// 遍历有序字典，打印出端口名和对应的信号
	f << "    Inputs:\n";
	for (const auto &pair : info.ordered_inputs) {
		f << "      ." << pair.first << ": " << log_signal(pair.second) << "\n";
	}

	string init_hex_str = format_init_hex(info.init_val);
	f << "    INIT: " << GetSize(info.init_val) << "'h" << init_hex_str << "\n";
	f << "    INIT: " << GetSize(info.init_val) << "'b" << info.init_val.as_string() << "\n\n";
}

//将每个 LUT 的详细信息写入文件
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
// 步骤 1: 提取所有单输出GTP_LUT的信息
//LUT的输入信号
//LUT的输出信号
//LUT的大小
//LUT的内存地址
//LUT的INIT
// =================================================================
void CollectLuts(Module *module, SigMap &sigmap, vector<LutInfo> &luts)
{
	//防止旧数据干扰
	luts.clear();
	//遍历
	for (Cell *cell : module->cells()) {
		//筛选出LUT
		const char *type_str = cell->type.c_str();
		if (strncmp(type_str, "\\GTP_LUT", 8) == 0 && strlen(type_str) == 9) {
			LutInfo info;
			info.cell_ptr = cell;				//保存指针操作数据
			info.size = type_str[8] - '0';		//LUT大小
			
			//如果某个端口没有连接信号线
			//TODO
			for (int i = 0; i < info.size; ++i) {
				string port_name = "I" + to_string(i);
				IdString port_id = IdString("\\" + port_name); 

				if (cell->hasPort(port_id)) {
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

//寻找可以合并的LUT/存储在candidates里面
void FindMergeCandidates(const vector<LutInfo> &luts, priority_queue<MergeCandidate> &candidates)
{
	//遍历LUT
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

			//两种情况
			//第一种	输入总和小于等于5
			if (current_union_inputs.size() <= 5) {
				int score = shared_inputs * 100 - current_union_inputs.size();
				candidates.push({(int)i, (int)j, score, current_union_inputs});
			}

			//TODO	
			//第二种	输入等于6
			//先检查是否可以合并，再放进candidates
		}
	}
}

// =================================================================
// 步骤 3: 执行合并操作
// 用于存储合并计划的安全结构体，不包含任何实时指针
// =================================================================
struct MergePlan {
	RTLIL::IdString new_cell_name;		//生成新LUT的名字
	RTLIL::Const init_val;				//生成新LUT的真值表
	map<RTLIL::IdString, RTLIL::SigBit> port_connections;	//生成新LUT的信号
	RTLIL::IdString cell_a_to_remove; // 只存储名字
	RTLIL::IdString cell_b_to_remove; // 只存储名字
};

//此函数只负责规划，返回一个安全的计划列表
//目前只讨论最简单的情况
//TODO	补充复杂情况
vector<MergePlan> PlanMerges(Module *module, vector<LutInfo> &luts, priority_queue<MergeCandidate> &candidates)
{
	vector<MergePlan> plans;

	while (!candidates.empty()) {
		//找到最合适的LUT对
		MergeCandidate best_pair = candidates.top();
		candidates.pop();

		LutInfo &lut_a = luts[best_pair.idx_a];
		LutInfo &lut_b = luts[best_pair.idx_b];

		//若ab已经被其它对占用，忽视
		if (lut_a.is_merged || lut_b.is_merged)
			continue;
		//否则标记
		lut_a.is_merged = true;
		lut_b.is_merged = true;

		//将之前存储的输入和作为新的输入
		vector<SigBit> new_inputs_vec;

		//TODO	将LUTa输入顺序填写	再将LUTb特有的部分顺序填写
		new_inputs_vec.push_back();

		//若输入小于5,补齐
		while (new_inputs_vec.size() < 6)
			new_inputs_vec.push_back(RTLIL::S0);
		//第6个输入sel固定为常数0
		new_inputs_vec.push_back(RTLIL::S0);

		//保存ab的输入
		pool<SigBit> inputs_a, inputs_b;
		for (const auto &p : lut_a.ordered_inputs)
			inputs_a.insert(p.second);
		for (const auto &p : lut_b.ordered_inputs)
			inputs_b.insert(p.second);

		//TODO，新输入的最后一个就是sel，没必要单独传，不需要调整位置
		// auto sel_it = find(new_inputs_vec.begin(), new_inputs_vec.end(), sel_bit);
		// if (sel_it != new_inputs_vec.end())
		// 	iter_swap(sel_it, new_inputs_vec.end() - 1);

		//TODO，之前没有保存两个LUT的输出马
		// SigBit z_out_target, z5_out_target;
		SigBit z_out_target = lut_b.output;
		SigBit z5_out_target = lut_a.output;

		//TODO sel 没有必要记录
		RTLIL::Const new_init = calculate_new_init(lut_a, lut_b, new_inputs_vec, sel_bit, z_out_target, z5_out_target);

		//创建一个结构体，保存合并LUT所需要的全部信息
		MergePlan plan;
		//新LUT 名字 a + b + merged
		string new_name_str = string(log_id(lut_a.cell_ptr->name)) + "_" + string(log_id(lut_b.cell_ptr->name)) + "_merged";
		plan.new_cell_name = module->uniquify(RTLIL::IdString("\\" + new_name_str));
		//新LUT 结构体真值表
		plan.init_val = new_init;
		//新LUT 输入信号
		for (size_t k = 0; k < 6; ++k)
			plan.port_connections[IdString("\\I" + to_string(k))] = new_inputs_vec[k];
		//新LUT 输出
		plan.port_connections[ID(Z)] = z_out_target;
		plan.port_connections[ID(Z5)] = z5_out_target;

		//需要移除的LUT
		plan.cell_a_to_remove = lut_a.cell_ptr->name;
		plan.cell_b_to_remove = lut_b.cell_ptr->name;

		//保存规划
		plans.push_back(plan);
	}
	return plans;
}

//合并流程
void StitcherMain(Module *module, const std::string &dump_filename)
{
	//计算用时
	auto start_time = std::chrono::high_resolution_clock::now();

	//将所有单输入的LUT读入内存
	log("Step 1: Scanning and collecting all GTP_LUTs...\n");

    // 建立信号关联数据库：将这些连接关系按 “逐位”（bit-by-bit）的方式记录到 SigMap 的内部数据库（database）中，形成一个全局的信号映射表。
	SigMap sigmap(module);
	vector<LutInfo> all_luts;
    // 遍历模块所有连接：扫描 module->connections() 中存储的所有信号连接关系（导线之间的连接、端口与内部信号的连接）
	CollectLuts(module, sigmap, all_luts);

	//TODO 什么file
	if (!dump_filename.empty()) {
		dump_luts_to_file(dump_filename, all_luts);
	}

	//寻找合适的LUT对
	log("Step 2: Finding best pairs to merge...\n");
	priority_queue<MergeCandidate> candidates;
	FindMergeCandidates(all_luts, candidates);
	log("Found %zu potential merge candidates.\n", candidates.size());

	// --- 阶段一: 规划合并方案 ---
	vector<MergePlan> plans = PlanMerges(module, all_luts, candidates);
	if (plans.empty()) {
		log("No valid merges found.\n");
		return;
	}
	log("Planned %zu merges.\n", plans.size());

	// --- 阶段二: 执行合并方案 (安全的"先移除，后添加"策略) ---
	pool<RTLIL::IdString> cell_names_to_remove;
	//将所有等待删除的旧LUT记录
	for (const auto &plan : plans) {
		cell_names_to_remove.insert(plan.cell_a_to_remove);
		cell_names_to_remove.insert(plan.cell_b_to_remove);
	}
	//遍历记录	调用API删除
	for (auto cell_name : cell_names_to_remove) {
		if (module->cell(cell_name)) {
			module->remove(module->cell(cell_name));
		}
	}

	for (const auto &plan : plans) {
		//创建GTP_LUT6D
		Cell *new_lut = module->addCell(plan.new_cell_name, ID(GTP_LUT6D));
		//设置真值表
		new_lut->setParam(ID::INIT, plan.init_val);
		//设置新的输入，输出
		for (const auto &conn : plan.port_connections) {
			new_lut->setPort(conn.first, conn.second);
		}
	}
	// 输出合并成功的数量
	log("Performed %zu merges successfully.\n", plans.size());
	// 计算并输出总耗时
	auto end_time = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> elapsed_ms = end_time - start_time;
	log("Whole process took %.2f ms.\n", elapsed_ms.count());
}

//命令注册
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