//g++ -std=c++17 -o truth_table_shuffler truth_table_shuffler.cpp
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <cmath>
#include <stdexcept>

// 将十六进制字符转换为4位二进制字符串
std::string hex_char_to_bin(char c) {
    switch (toupper(c)) {
        case '0': return "0000"; case '1': return "0001";
        case '2': return "0010"; case '3': return "0011";
        case '4': return "0100"; case '5': return "0101";
        case '6': return "0110"; case '7': return "0111";
        case '8': return "1000"; case '9': return "1001";
        case 'A': return "1010"; case 'B': return "1011";
        case 'C': return "1100"; case 'D': return "1101";
        case 'E': return "1110"; case 'F': return "1111";
        default: throw std::invalid_argument("Invalid hex character");
    }
}

// 将二进制字符串转换为十六进制字符串
std::string bin_to_hex(const std::string& bin) {
    if (bin.empty()) return "0";
    std::string padded_bin = bin;
    while (padded_bin.length() % 4 != 0) {
        padded_bin.insert(0, "0");
    }

    std::string hex;
    const char hex_chars[] = "0123456789ABCDEF";
    for (size_t i = 0; i < padded_bin.length(); i += 4) {
        int nibble = 0;
        if (padded_bin[i] == '1') nibble += 8;
        if (padded_bin[i + 1] == '1') nibble += 4;
        if (padded_bin[i + 2] == '1') nibble += 2;
        if (padded_bin[i + 3] == '1') nibble += 1;
        hex += hex_chars[nibble];
    }
    return hex;
}


// 解析输入的真值表字符串（支持 'b' 和 'h' 前缀）
std::string parse_truth_table(const std::string& raw_tt, int n_bits) {
    if (raw_tt.length() < 2) {
        throw std::invalid_argument("Truth table string is too short.");
    }

    char format = tolower(raw_tt[0]);
    std::string value = raw_tt.substr(1);
    std::string bin_tt;

    if (format == 'b') {
        bin_tt = value;
    } else if (format == 'h') {
        for (char c : value) {
            bin_tt += hex_char_to_bin(c);
        }
    } else {
        throw std::invalid_argument("Invalid truth table format. Use 'b' or 'h'.");
    }

    // Yosys的INIT最低位在前，但通常我们读写时最高位在前，所以翻转一下
    std::reverse(bin_tt.begin(), bin_tt.end());
    
    size_t expected_length = 1 << n_bits;
    if (bin_tt.length() < expected_length) {
        bin_tt.append(expected_length - bin_tt.length(), '0'); // 补0
    } else if (bin_tt.length() > expected_length) {
        bin_tt = bin_tt.substr(0, expected_length); // 截断
    }

    return bin_tt;
}

int main() {
    try {
        // --- 1. 输入 ---
        std::string original_weights_str;
        std::string original_tt_str;
        std::string new_weights_str;

        std::cout << "Enter original weight relationship (e.g., ABCD): ";
        std::cin >> original_weights_str;

        std::cout << "Enter original truth table (e.g., b11011001 or hD9): ";
        std::cin >> original_tt_str;
        
        std::cout << "Enter new weight relationship (e.g., BCDA): ";
        std::cin >> new_weights_str;

        if (original_weights_str.length() != new_weights_str.length()) {
            throw std::runtime_error("Weight strings must have the same length.");
        }

        int n_vars = original_weights_str.length();
        size_t tt_size = 1 << n_vars;

        // --- 2. 解析和准备 ---
        std::string original_tt_bin = parse_truth_table(original_tt_str, n_vars);
        std::string new_tt_bin(tt_size, '0');

        // 创建从新权重字符到旧权重字符位置的映射
        std::map<char, int> old_weights_map;
        for(int i = 0; i < n_vars; ++i) {
            old_weights_map[original_weights_str[i]] = i;
        }

        std::cout << "\n--- Processing ---\n";
        std::cout << "Original Weights: " << original_weights_str << std::endl;
        std::cout << "New Weights:      " << new_weights_str << std::endl;

        // --- 3. 核心转换逻辑 ---
        for (size_t new_addr = 0; new_addr < tt_size; ++new_addr) {
            size_t old_addr = 0;
            // 遍历新的位权关系的每一位
            for (int i = 0; i < n_vars; ++i) {
                // 检查在新地址中，当前位(i)是否为1
                if ((new_addr >> i) & 1) {
                    // 如果是1，找到这个位(i)对应的新权重字符
                    char new_weight_char = new_weights_str[i];
                    // 查表，找到这个字符在旧权重关系中的位置（权重）
                    int old_pos = old_weights_map.at(new_weight_char);
                    // 将这个旧的权重累加到旧地址中
                    old_addr += (1 << old_pos);
                }
            }

            // 从原始真值表中取出值，赋给新真值表
            new_tt_bin[new_addr] = original_tt_bin[old_addr];
        }

        // --- 4. 输出 ---
        // 翻转回来以便于阅读
        std::reverse(new_tt_bin.begin(), new_tt_bin.end());

        std::cout << "\n--- Results ---\n";
        std::cout << "New Truth Table (binary): " << new_tt_bin << std::endl;
        std::cout << "New Truth Table (hex):    " << bin_to_hex(new_tt_bin) << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}