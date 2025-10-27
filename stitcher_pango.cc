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
#include <omp.h>

#define Layered // 控制是否启用分层优化

USING_YOSYS_NAMESPACE
using namespace std;
PRIVATE_NAMESPACE_BEGIN

// =================================================================
// 计时辅助类
// =================================================================
class ScopedTimer
{
      public:
	// 构造函数：记录起始时间并打印开始信息
	ScopedTimer(const std::string &task_name) : task_name_(task_name), start_time_(std::chrono::high_resolution_clock::now())
	{
		log("\n--- Timing start: %s ---\n", task_name_.c_str());
	}

	// 析构函数：记录结束时间，计算差值，并打印结果
	~ScopedTimer()
	{
		auto end_time = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time_).count();
		log("--- Timing end: %s | Duration: %.3f seconds ---\n", task_name_.c_str(), duration / 1000.0);
	}

      private:
	std::string task_name_;
	std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
};

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

// 定义动态选择策略的阈值
// 如果LUT总数超过这个值，就启用分层优化
const size_t LAYERED_SEARCH_THRESHOLD = 30000;

// 用于按层次存储LUT索引的地图
map<int, vector<int>> level_to_lut_indices;

// =================================================================
// 新增：根据拓扑深度对LUT进行分层
// =================================================================

// 用于存储每个LUT的深度/层次信息
dict<RTLIL::Cell *, int> lut_levels;

// 分层函数
void NumberLutsByLevel(const vector<LutInfo> &luts)
{
	log("Numbering LUTs by topological level...\n");
	lut_levels.clear();

	// --- 准备工作：建立图的邻接关系和入度表 ---
	dict<RTLIL::Cell *, pool<RTLIL::Cell *>> lut_graph; // LUT -> set of downstream LUTs
	dict<RTLIL::Cell *, int> lut_in_degree;
	dict<RTLIL::SigBit, RTLIL::Cell *> output_sig_to_lut; // 方便快速查找驱动LUT

	// 快速查找一个Cell指针是否在我们的LUT列表中
	pool<RTLIL::Cell *> lut_cell_pool;
	for (const auto &lut : luts) {
		lut_cell_pool.insert(lut.cell_ptr);
		output_sig_to_lut[lut.output] = lut.cell_ptr;

		lut_graph[lut.cell_ptr] = {};
	}

	for (const auto &src_lut : luts) {
		lut_in_degree[src_lut.cell_ptr] = 0; // 初始化入度

		// 计算当前LUT的入度 (只考虑来自其他LUT的输入)
		for (const auto &pair : src_lut.ordered_inputs) {
			SigBit input_sig = pair.second;
			// 检查这个输入是否由另一个LUT驱动
			if (output_sig_to_lut.count(input_sig)) {
				RTLIL::Cell *driver_lut_ptr = output_sig_to_lut.at(input_sig);

				// 建立从驱动LUT到当前LUT的边
				if (lut_graph[driver_lut_ptr].insert(src_lut.cell_ptr).second) {
					// 只有在第一次添加边时才增加入度
					lut_in_degree[src_lut.cell_ptr]++;
				}
			}
		}
	}

	// --- 拓扑排序计算深度 (Kahn's Algorithm using BFS) ---
	queue<RTLIL::Cell *> q;
	for (const auto &lut : luts) {
		if (lut_in_degree[lut.cell_ptr] == 0) {
			q.push(lut.cell_ptr);
			lut_levels[lut.cell_ptr] = 0; // 起始LUT的深度为0
		}
	}

	int processed_count = 0;
	while (!q.empty()) {
		RTLIL::Cell *current_lut_ptr = q.front();
		q.pop();
		processed_count++;

		int current_level = lut_levels.at(current_lut_ptr);

		for (RTLIL::Cell *downstream_lut_ptr : lut_graph.at(current_lut_ptr)) {
			// 更新下游节点的深度为 max(current_depth, my_depth + 1)
			int new_level = current_level + 1;
			if (!lut_levels.count(downstream_lut_ptr) || new_level > lut_levels.at(downstream_lut_ptr)) {
				lut_levels[downstream_lut_ptr] = new_level;
			}

			// 更新入度
			if (--lut_in_degree.at(downstream_lut_ptr) == 0) {
				q.push(downstream_lut_ptr);
			}
		}
	}

	if (processed_count != (int)luts.size()) {
		log_warning("Combinational loop detected among LUTs. Leveling may be incomplete.\n");
		// 对于循环中的节点，它们不会被处理，level将不存在
	}
}

#pragma region print_funcs
// =================================================================
// 新增的打印辅助函数
// =================================================================
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
#pragma endregion print_funcs

RTLIL::Const calculate_new_init(const LutInfo &lut_a, const LutInfo &lut_b, const vector<SigBit> &new_inputs_vec, const SigBit &sel_bit,
				SigBit &z_out_sig, SigBit &z5_out_sig)
{
	// --- 准备工作 ---
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
	for (int i = 0; i < 32; ++i) { // 遍历I[4:0]的所有32种组合
		size_t addr_a = 0;     // 原始lut_for_z5中的地址
		int bit_pos = 1;       // 对应原始lut_for_z5中输入端口的权重

		// 遍历 lut_for_z5 的每一个原始输入端口
		for (const auto &pair : lut_for_z5.ordered_inputs) {
			// 在新LUT的I[4:0]输入中查找这个信号
			auto it = find(shared_inputs.begin(), shared_inputs.end(), pair.second);
			if (it != shared_inputs.end()) {
				int shared_idx = distance(shared_inputs.begin(), it);
				// 如果在新LUT的当前组合'i'中，这个输入位是'1'
				if ((i >> shared_idx) & 1)
					addr_a += bit_pos; // 累加地址
			}
			// 如果找不到，说明这个输入是sel_bit，在I5=0的情况下，它贡献的地址值为0，所以什么都不用做
			bit_pos <<= 1;
		}

		if (addr_a < (size_t)GetSize(init_a_copy)) {
			z5_bits.push_back(init_a_copy.bits().at(addr_a)); // 从原始INIT中查找并存入
		} else {
			z5_bits.push_back(RTLIL::S0); // 安全兜底
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

#pragma region complex_case_funcs
// 将 Const 转换为 uint64_t 以便进行位运算
uint64_t const_to_uint64(const RTLIL::Const &c)
{
	// 创建一个非const的拷贝，现在可以在它上面安全地调用非const成员函数
	RTLIL::Const c_copy = c;

	uint64_t val = 0;
	// Yosys 的 Const bit[0] 是最低位
	for (int i = 0; i < c_copy.size() && i < 64; ++i) {
		if (c_copy.bits().at(i) == RTLIL::S1) { // 现在调用是合法的
			val |= (1ULL << i);
		}
	}
	return val;
}

// 辅助函数，生成输入信号在64位真值表中的掩码
uint64_t get_input_mask(const map<string, SigBit> &ordered_inputs_6, const SigBit &target_sig)
{
	// 模板掩码
	const uint64_t masks[] = {
	  0xAAAAAAAAAAAAAAAA, // I0
	  0xCCCCCCCCCCCCCCCC, // I1
	  0xF0F0F0F0F0F0F0F0, // I2
	  0xFF00FF00FF00FF00, // I3
	  0xFFFF0000FFFF0000, // I4
	  0xFFFFFFFF00000000  // I5
	};
	int input_idx = 0;
	for (const auto &pair : ordered_inputs_6) {
		if (pair.second == target_sig) {
			return masks[input_idx];
		}
		input_idx++;
	}
	return 0; // Not found
}

// 仅考虑了子LUT为5输入时的情况，缺乏INIT扩充代码
bool CanLut6AbsorbLutS(const LutInfo &lut_6, const LutInfo &lut_s, SigBit &found_sel_bit)
{
	// --- 1. 找到潜在的 sel_bit ---
	// sel_bit 是 lut_6 的输入，但不是 lut_s 的输入
	vector<SigBit> potential_sel_bits;
	pool<SigBit> inputs_s;
	for (const auto &p : lut_s.ordered_inputs)
		inputs_s.insert(p.second);

	for (const auto &p_6 : lut_6.ordered_inputs) {
		if (!inputs_s.count(p_6.second)) {
			potential_sel_bits.push_back(p_6.second);
		}
	}

	// 对于 LUT6 吸收 LUTs (s<6)，必须恰好有一个非共享输入作为 sel_bit
	if (potential_sel_bits.size() != 1) {
		return false;
	}
	found_sel_bit = potential_sel_bits[0];

	// --- 2. 扩展 lut_s 的 INIT 到 64 位 ---
	// 这个过程是根据输入信号的映射关系来“复制”位
	uint64_t s_expanded_tt = 0;
	for (int i = 0; i < (1 << lut_s.size); ++i) {
		if ((const_to_uint64(lut_s.init_val) >> i) & 1) {
			// 对于 lut_s 真值表中为'1'的每一行，我们需要在64位空间中找到所有对应的位置并置'1'
			uint64_t target_mask = 0xFFFFFFFFFFFFFFFF;
			int s_input_idx = 0;
			for (const auto &p_s : lut_s.ordered_inputs) {
				uint64_t lut6_mask = get_input_mask(lut_6.ordered_inputs, p_s.second);
				if ((i >> s_input_idx) & 1) {
					target_mask &= lut6_mask; // 该位为1，保留mask
				} else {
					target_mask &= ~lut6_mask; // 该位为0，保留mask的反
				}
				s_input_idx++;
			}
			s_expanded_tt |= target_mask;
		}
	}

	// --- 3. 模板匹配验证 ---
	uint64_t sel_mask = get_input_mask(lut_6.ordered_inputs, found_sel_bit);
	uint64_t tt_6 = const_to_uint64(lut_6.init_val);

	// 检查当 sel=0 时，lut_6 的逻辑是否与扩展后的 lut_s 逻辑相同
	// 我们只关心 sel_mask 中为'0'的那些位
	bool sel_is_0_match = ((tt_6 & ~sel_mask) == (s_expanded_tt & ~sel_mask));

	// 检查当 sel=1 时，lut_6 的逻辑是否与扩展后的 lut_s 逻辑相同
	// 我们只关心 sel_mask 中为'1'的那些位
	bool sel_is_1_match = ((tt_6 & sel_mask) == (s_expanded_tt & sel_mask));

	// 只要其中一种情况匹配，就可以合并
	// 如果 sel=0 匹配，那么 lut_s 的逻辑可以放在 Z5 端口
	// 如果 sel=1 匹配，那么 lut_s 的逻辑可以放在 Z 端口
	return sel_is_0_match || sel_is_1_match;
}
#pragma endregion complex_case_funcs

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
			luts.push_back(info); // 此处自然地为每个LUT标上唯一序号(index)
		}
	}
}

// =================================================================
// 步骤 2: 寻找并评估所有可合并的候选对
// =================================================================
enum class MergeType {
	SHARED_INPUTS, // 总输入<=5
	LUT6_ABSORB    // LUT6吸收小LUT
};

struct MergeCandidate {
	int idx_a, idx_b;
	int score;
	pool<SigBit> union_inputs;

	MergeType type;
	SigBit discovered_sel_bit; // 仅在 LUT6_ABSORB 类型下有效

	bool operator<(const MergeCandidate &other) const { return score < other.score; }
};

// 已添加分层逻辑
void FindMergeCandidates_Layered(const vector<LutInfo> &luts, priority_queue<MergeCandidate> &candidates)
{
	log("Using layered search strategy (LUT count > %zu).\n", LAYERED_SEARCH_THRESHOLD);

	// --- 辅助Lambda 1: 检查 SHARED_INPUTS 类型的合并 ---
	auto check_shared_inputs = [&](int i, int j) {
		const LutInfo &lut_a = luts[i];
		const LutInfo &lut_b = luts[j];

		pool<SigBit> current_union_inputs;
		for (const auto &pair : lut_a.ordered_inputs)
			current_union_inputs.insert(pair.second);
		for (const auto &pair : lut_b.ordered_inputs)
			current_union_inputs.insert(pair.second);

		if (current_union_inputs.size() <= 5) {
			int shared_inputs = (lut_a.size + lut_b.size) - current_union_inputs.size();
			if (shared_inputs >= 1) {
				int score = shared_inputs * 100 - current_union_inputs.size();
				candidates.push({i, j, score, current_union_inputs, MergeType::SHARED_INPUTS, RTLIL::Sx});
			}
		}
	};

	// --- 辅助Lambda 2: 检查 LUT6_ABSORB 类型的合并 ---
	auto check_lut6_absorb = [&](int idx_6, int idx_s) {
		const LutInfo &lut_6 = luts[idx_6];
		const LutInfo &lut_s = luts[idx_s];

		// 1. 快速检查：尺寸必须是6和一个更小的
		if (lut_6.size != 6 || lut_s.size >= 6)
			return;

		// 2. 检查输入子集关系
		pool<SigBit> inputs_6, inputs_s;
		for (const auto &p : lut_6.ordered_inputs)
			inputs_6.insert(p.second);
		for (const auto &p : lut_s.ordered_inputs)
			inputs_s.insert(p.second);

		bool is_subset = true;
		for (const auto &sig_s : inputs_s) {
			if (!inputs_6.count(sig_s)) {
				is_subset = false;
				break;
			}
		}
		if (!is_subset)
			return;

		// 3. 检查逻辑关系
		SigBit sel_bit;
		if (CanLut6AbsorbLutS(lut_6, lut_s, sel_bit)) {
			int score = 10000 + lut_s.size * 100;
			candidates.push({idx_6, idx_s, score, inputs_6, MergeType::LUT6_ABSORB, sel_bit});
		}
	};

	// --- 核心：分层搜索 ---
	for (const auto &pair : level_to_lut_indices) {
		int level = pair.first;
		const vector<int> &luts_in_current_level = pair.second;

		// --- 1. 在当前层内部进行搜索 ---
		for (size_t i = 0; i < luts_in_current_level.size(); ++i) {
			for (size_t j = i + 1; j < luts_in_current_level.size(); ++j) {
				int lut_idx_a = luts_in_current_level[i];
				int lut_idx_b = luts_in_current_level[j];

				// 检查两种合并可能性
				check_shared_inputs(lut_idx_a, lut_idx_b);
				check_lut6_absorb(lut_idx_a, lut_idx_b); // 检查 a 是否能吸收 b
				check_lut6_absorb(lut_idx_b, lut_idx_a); // 检查 b 是否能吸收 a
			}
		}

		// --- 2. 与下一相邻层之间进行搜索 ---
		if (level_to_lut_indices.count(level + 1)) {
			const vector<int> &luts_in_next_level = level_to_lut_indices.at(level + 1);
			for (int lut_idx_curr : luts_in_current_level) {
				for (int lut_idx_next : luts_in_next_level) {
					// 检查两种合并可能性
					check_shared_inputs(lut_idx_curr, lut_idx_next);
					check_lut6_absorb(lut_idx_curr, lut_idx_next); // 检查 curr 是否能吸收 next
					check_lut6_absorb(lut_idx_next, lut_idx_curr); // 检查 next 是否能吸收 curr
				}
			}
		}
	}
}

void FindMergeCandidates_Global(const vector<LutInfo> &luts, priority_queue<MergeCandidate> &candidates)
{
	log("Using global search strategy (LUT count <= %zu).\n", LAYERED_SEARCH_THRESHOLD);

	//  --- 第一部分：处理总输入 <= 5 的情况 (代码保持不变) ---
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
					candidates.push({(int)i, (int)j, score, current_union_inputs, MergeType::SHARED_INPUTS, RTLIL::Sx});
				}
			}
		}
	}
	//	--- 第二部分：处理 LUT6 吸收小 LUT 的情况 ---
	for (size_t i = 0; i < luts.size(); ++i) {
		if (luts[i].size != 6)
			continue; // 只从 LUT6 开始
		const LutInfo &lut_6 = luts[i];

		for (size_t j = 0; j < luts.size(); ++j) {
			if (i == j)
				continue;
			const LutInfo &lut_s = luts[j];

			// 1. 检查输入子集关系
			pool<SigBit> inputs_6, inputs_s;
			for (const auto &p : lut_6.ordered_inputs)
				inputs_6.insert(p.second);
			for (const auto &p : lut_s.ordered_inputs)
				inputs_s.insert(p.second);

			bool is_subset = true;
			for (const auto &sig_s : inputs_s) {
				if (!inputs_6.count(sig_s)) {
					is_subset = false;
					break;
				}
			}
			if (!is_subset)
				continue;

			// 2. 检查逻辑关系 (调用新的辅助函数)
			SigBit sel_bit;
			if (CanLut6AbsorbLutS(lut_6, lut_s, sel_bit)) {
				// 这是一个完美的吸收机会！
				// 给予极高的分数，确保优先处理
				int score = 10000 + lut_s.size * 100;
				candidates.push({(int)i, (int)j, score, inputs_6, MergeType::LUT6_ABSORB, sel_bit});
			}
		}
	}
}

// =================================================================
// 步骤 3: 执行合并操作
// 用于存储合并计划的安全结构体，不包含任何实时指针
// =================================================================
struct MergePlan {
	RTLIL::IdString new_cell_name;			      // 生成新LUT的名字
	RTLIL::Const init_val;				      // 生成新LUT的真值表
	map<RTLIL::IdString, RTLIL::SigBit> port_connections; // 生成新LUT的信号
	RTLIL::IdString cell_a_to_remove;		      // 只存储名字
	RTLIL::IdString cell_b_to_remove;		      // 只存储名字
};

// 此函数只负责规划，返回一个安全的计划列表
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

		vector<SigBit> new_inputs_vec;
		SigBit sel_bit;

		// --- 【核心修改：根据类型分发】 ---
		if (best_pair.type == MergeType::SHARED_INPUTS) {
			// --- 情况一：总输入数 <= 5 ---
			for (const auto &sig : best_pair.union_inputs)
				new_inputs_vec.push_back(sig);
			// 补齐空输入（当总输入数小于5时）
			while (new_inputs_vec.size() < 5)
				new_inputs_vec.push_back(RTLIL::S0);
			// 最后的一位信号为常数1作为sel_bit
			sel_bit = RTLIL::S1;
			new_inputs_vec.push_back(sel_bit);
		} else { // best_pair.type == MergeType::LUT6_ABSORB
			// --- 情况二：LUT6 吸收小 LUT ---
			const LutInfo &lut_6 = (lut_a.size == 6) ? lut_a : lut_b;

			// 1. 新的输入向量就是LUT6的原始输入向量
			for (const auto &pair : lut_6.ordered_inputs) {
				new_inputs_vec.push_back(pair.second);
			}

			// 2. sel_bit 是之前已经发现并存储好的
			sel_bit = best_pair.discovered_sel_bit;

			// 3. 端口映射：确保 sel_bit 在最后一位
			auto sel_it = find(new_inputs_vec.begin(), new_inputs_vec.end(), sel_bit);
			if (sel_it != new_inputs_vec.end())
				iter_swap(sel_it, new_inputs_vec.end() - 1);
		}

		// --- 公共逻辑：计算 INIT 并创建规划 ---
		SigBit z_out_target, z5_out_target;

		RTLIL::Const new_init = calculate_new_init(lut_a, lut_b, new_inputs_vec, sel_bit, z_out_target, z5_out_target);

		// 创建一个结构体，保存合并LUT所需要的全部信息
		MergePlan plan;
		// 新LUT 名字 a + b + merged
		string new_name_str = string(log_id(lut_a.cell_ptr->name)) + "_" + string(log_id(lut_b.cell_ptr->name)) + "_merged";
		plan.new_cell_name = module->uniquify(RTLIL::IdString("\\" + new_name_str));
		// 新LUT 结构体真值表
		plan.init_val = new_init;
		// 新LUT 输入信号
		for (size_t k = 0; k < 6; ++k)
			plan.port_connections[IdString("\\I" + to_string(k))] = new_inputs_vec[k];
		// 新LUT 输出
		plan.port_connections[ID(Z)] = z_out_target;
		plan.port_connections[ID(Z5)] = z5_out_target;
		// 需要移除的LUT
		plan.cell_a_to_remove = lut_a.cell_ptr->name;
		plan.cell_b_to_remove = lut_b.cell_ptr->name;

		// 保存规划
		plans.push_back(plan);
	}
	return plans;
}

// 合并流程
void StitcherMain(Module *module, const std::string &dump_filename)
{
	// --- 总计时器 ---
	ScopedTimer total_timer("Total StitcherMain Execution");

	// === 步骤 1: 收集LUTs ===
	vector<LutInfo> all_luts;
	{ // 使用花括号创建一个新的作用域
		ScopedTimer step1_timer("Step 1: CollectLuts");
		SigMap sigmap(module);
		CollectLuts(module, sigmap, all_luts);

		if (!dump_filename.empty()) {
			dump_luts_to_file(dump_filename, all_luts);
		}
	} // step1_timer 在这里离开作用域，自动打印时间

	log("Collected %zu LUTs.\n", all_luts.size());

	// === 步骤 2: 计算深度并分区 ===

	// === 【核心决策逻辑】 ===
	priority_queue<MergeCandidate> candidates;

	if (all_luts.size() <= LAYERED_SEARCH_THRESHOLD) {
		// --- 路径A：LUT数量少，使用快速的全局搜索 ---

		// 分层是不必要的，但我们仍然可以打印一条信息
		log("Skipping leveling/partitioning for small design.\n");

		{
			ScopedTimer step3_timer("Step 3: FindMergeCandidates (Global)");
			FindMergeCandidates_Global(all_luts, candidates);
			log("Found %zu potential merge candidates.\n", candidates.size());
		}

	} else {
		// --- 路径B：LUT数量多，必须使用分层优化 ---

		// === 步骤 2: 计算深度并分区 ===
		int max_level = 0;
		{
			ScopedTimer step2_timer("Step 2: NumberLutsByLevel & Partition");
			NumberLutsByLevel(all_luts);

			level_to_lut_indices.clear();
			int max_level = 0;
			for (size_t i = 0; i < all_luts.size(); ++i) {
				RTLIL::Cell *cell_ptr = all_luts[i].cell_ptr;
				if (lut_levels.count(cell_ptr)) {
					int level = lut_levels.at(cell_ptr);
					level_to_lut_indices[level].push_back(i);
					if (level > max_level)
						max_level = level;
				}
			}

			log("Partitioned LUTs into %zu levels (from 0 to %d).\n", level_to_lut_indices.size(), max_level);
		}

		// === 步骤 3: 寻找候选对 ===
		{
			ScopedTimer step3_timer("Step 3: FindMergeCandidates (Layered)");
			FindMergeCandidates_Layered(all_luts, candidates);
			log("Found %zu potential merge candidates.\n", candidates.size());
		}
	}

	// === 步骤 4: 规划合并方案 ===
	vector<MergePlan> plans;
	{
		ScopedTimer step4_timer("Step 4: PlanMerges");
		plans = PlanMerges(module, all_luts, candidates);
		if (plans.empty()) {
			log("No valid merges found.\n");
			// 函数提前结束，total_timer 会自动析构并打印总时间
			return;
		}
		log("Planned %zu merges.\n", plans.size());
	}

	// === 步骤 5: 执行合并方案 ===
	{
		ScopedTimer step5_timer("Step 5: ExecuteMerges (remove & add cells)");
		pool<RTLIL::IdString> cell_names_to_remove;
		for (const auto &plan : plans) {
			cell_names_to_remove.insert(plan.cell_a_to_remove);
			cell_names_to_remove.insert(plan.cell_b_to_remove);
		}
		for (auto cell_name : cell_names_to_remove) {
			if (module->cell(cell_name)) {
				module->remove(module->cell(cell_name));
			}
		}

		for (const auto &plan : plans) {
			Cell *new_lut = module->addCell(plan.new_cell_name, ID(GTP_LUT6D));
			new_lut->setParam(ID::INIT, plan.init_val);
			for (const auto &conn : plan.port_connections) {
				new_lut->setPort(conn.first, conn.second);
			}
		}
		log("Performed %zu merges successfully.\n", plans.size());
	}

	// total_timer 在函数末尾离开作用域，自动打印总时间
}

// 命令注册
struct StitcherPass : public Pass {
	StitcherPass() : Pass("stitcher", "Basic Task: find and stitch GTP_LUTs.") {}

	std::string dump_filename;
	void clear_flags() override { dump_filename = ""; }

	void execute(vector<string> args, RTLIL::Design *design) override
	{
		log_header(design, "Executing StitcherPass (Basic Task).\n");
		clear_flags();

		// 可添加-dump参数来把LUT结构体数组打印成一个文件
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