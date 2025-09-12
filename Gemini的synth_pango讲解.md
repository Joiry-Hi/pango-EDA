好的，我们来对`synth_pango.cc`这个核心文件进行一次全面的“代码导览”，解释每一个关键函数的作用，并最后总结出编译和映射的完整流程。

---

### **一、 `synth_pango.cc` 中各个函数的作用**

我们可以将这些函数按照功能逻辑分为几个类别：**Yosys接口、主控流程、图论与拓扑、核心映射算法、LUT生成工具**和**配置函数**。

#### **1. Yosys接口 (Passes)**

*   `struct MapperPass : public Pass`:
    *   **用途**: 在Yosys中注册一个名为`mapper`的新命令。
    *   **详解**: 这是你在`demo.ys`脚本中直接调用的命令。当Yosys执行`mapper`时，实际上就是调用了这个结构体中的`execute`方法，而`execute`方法的核心又是调用`MapperMain(module)`。它是你的算法在Yosys中的**直接入口点**。

*   `struct SynthPangoPass : public ScriptPass`:
    *   **用途**: 注册一个名为`synth_pango`的命令，这是一个**脚本式Pass**。
    *   **详解**: 它提供了一个更高级、封装好的综合流程。它内部会自己调用`read_verilog`, `hierarchy`以及`MapperMain`等一系列命令。你可以把它看作一个官方提供的、更完整的自动化脚本。在`demo.ys`中，你使用的是更底层的`mapper`命令，这给了你更大的灵活性。

#### **2. 主控流程 (Main Control Flow)**

*   `bool MapperMain(Module *module)`:
    *   **用途**: **算法的总指挥**。
    *   **详解**: 这是整个映射算法的顶层逻辑。它按照论文中的流程，依次调用各个子函数来完成任务：`GetTopoSortedGates`进行预处理，`GenerateCuts`进行预计算，然后在一个循环中多次调用`TraverseFWD`和`TraverseBWD`进行迭代优化，最后调用`ConeToLUTs`生成最终的电路。

*   `bool MapperInit(Module *module)`:
    *   **用途**: 初始化和清理。
    *   **详解**: 在`MapperMain`运行之前被调用，用于清空上一次运行时留下的所有全局变量（如`best_bit2cut`, `bit2driver`等），确保每次运行都是在一个干净的状态下开始。

#### **3. 图论与拓扑 (Graph & Topology Utilities)**

*   `bool CheckCellWidth(Module *module)`:
    *   **用途**: 遍历电路，构建核心的连接关系数据结构。
    *   **详解**: 这个函数的名字有点误导。它做的最重要的事情是遍历模块中所有的单元和连线，并填充`bit2driver`和`bit2reader`这两个字典。**这两个字典是整个算法能够遍历电路图的基础**，它们构成了电路的邻接表表示。

*   `void GetTopoSortedGates(Module *module, vector<Cell *> &gates)`:
    *   **用途**: 对电路中的所有组合逻辑门进行**拓扑排序**。
    *   **详解**: 拓扑排序确保了当算法处理任何一个逻辑门时，这个门的所有输入（上游节点）都已经被处理过了。这是所有基于动态规划或前向遍历的图算法能够正确工作的前提。

*   `bool GetPrimeInputOuput(...)`:
    *   **用途**: 找到整个组合逻辑部分的“边界”。
    *   **详解**: 它负责识别哪些信号线是整个待处理逻辑块的**主输入**（由寄存器或顶层端口驱动）和**主输出**（驱动寄存器或顶层端口）。`TraverseFWD`从这些主输入开始，`TraverseBWD`从这些主输出开始。

#### **4. 核心映射算法 (Core IMap Algorithm)**

*   `bool GenerateCuts(Cell *cell)` / `bool GenerateCuts(Module *module)`:
    *   **用途**: **预计算**。为每一个逻辑门，枚举出所有可能的、输入数不大于`LUT_SIZE`（6）的逻辑锥（割集）。
    *   **详解**: 这是算法中计算量最大的步骤之一。它为后续的决策过程准备好了所有“候选方案”。结果存储在`cell2cuts`中。

*   `bool TraverseFWD(...)`:
    *   **用途**: **前向遍历与决策**。
    *   **详解**: 按照拓扑序，从主输入到主输出，遍历所有逻辑门。在每个门上，它会调用`GetBestCut`来为这个门选择一个当前看来最优的LUT覆盖方案，并根据这个选择更新下游信号线的`depth`（深度）和`af`（面积流）成本。

*   `bool GetBestCut(Cell *cell, pool<SigBit> &cut_selected)`:
    *   **用途**: **算法的“大脑”**。
    *   **详解**: 这是启发式决策的核心。对于给定的`cell`，它会评估`cell2cuts`中所有的候选方案。根据当前的模式（深度优先或面积优先）和迭代轮次，它会利用`depth`和`af`这两个指标，选择一个成本最低的割集。**（如果你要实现双输出LUT映射，这里是你需要重点修改和扩展的地方）**。

*   `bool TraverseBWD(...)`:
    *   **用途**: **后向遍历与信息修正**。
    *   **详解**: 按照反向拓扑序，从主输出到主输入进行遍历。它的主要目的不是做决策，而是根据前向遍历的结果，更精确地更新每个节点的`height`（高度）和扇出估计值。这些被修正过的信息，将用于**下一轮**`TraverseFWD`中，使其做出更明智的决策。

#### **5. LUT生成工具 (LUT Generation)**

*   `bool ConeToLUTs(Module *module, dict<SigBit, pool<SigBit>> &bit2cut)`:
    *   **用途**: **代码生成**。将最终选择的映射方案（存储在`bit2cut`中）转换成真实的硬件。
    *   **详解**: 在所有迭代优化完成后，此函数被调用。它遍历最终确定的每一个逻辑锥，调用`addLut`为它们实例化一个`GTP_LUT`，最后删除所有旧的通用逻辑门。

*   `RTLIL::Cell *addLut(...)`:
    *   **用途**: 实例化一个`GTP_LUT`单元。
    *   **详解**: 这是实际创建新单元的“工人”函数。它会调用`GetCutInit`来计算LUT的`INIT`参数，然后创建一个`GTP_LUTx`实例并正确连接其输入输出端口。

*   `vector<bool> GetCutInit(...)`:
    *   **用途**: 计算LUT的**`INIT`初始化参数**。
    *   **详解**: 它会遍历一个割集所有`2^K`种输入组合，对每一种组合都调用`StateEval`来求解输出值，最终将这些值串联成`INIT`字符串。

*   `State StateEval(...)`:
    *   **用途**: 对一个逻辑锥进行**逻辑仿真**。
    *   **详解**: 给定割集输入的值，它会递归地计算出逻辑锥的输出值是0还是1。**（你的"unproven cells"错误，根源99%就在`GetCutInit`或`StateEval`的计算逻辑中）**。

#### **6. 配置函数**

*   `void SetPangoCellTypes(CellTypes *ct)`:
    *   **用途**: 向Yosys注册所有Pango原语的端口信息（输入/输出）。
    *   **详解**: 这是一段很长的、自动生成的代码。它告诉Yosys，像`GTP_LUT6`这样的单元，它的`I0`到`I5`是输入，`Z`是输出。这是Yosys能够正确处理这些原语的基础。你不需要修改它。

---

### **二、 流程总结**

#### **编译流程 (你的开发工作流)**

这是你在开发和调试时的操作步骤：

1.  **修改代码**: 你在你的代码编辑器中修改 `synth_pango.cc` 文件。
2.  **运行`make`**: 在Yosys的根目录下，你执行 `make` 或 `make -j$(nproc)`。
3.  **编译系统工作**:
    *   Yosys的主`Makefile`会找到`techlibs/pango/Makefile.inc`文件。
    *   这个`.inc`文件告诉`make`，需要用C++编译器（如g++或clang）将`synth_pango.cc`和`score.cc`编译成目标文件（`.o`）。
4.  **链接**: 编译器将所有新生成的目标文件与Yosys的其他部分链接在一起，生成一个**最终的可执行文件`yosys`**。
5.  **安装**: 你执行 `sudo make install`，这个命令会将新生成的`yosys`可执行文件复制到系统路径下（如`/usr/local/bin/`），覆盖掉旧版本。
6.  **完成**: 至此，你对代码的修改已经生效，可以运行`demo.ys`来测试了。

#### **映射流程 (Yosys运行`mapper`时的内部流程)**

这是当你运行`yosys -s demo.ys`时，`mapper`命令内部发生的事情：

1.  **初始化 (`MapperInit`)**: 清理所有全局变量，准备一个干净的运行环境。
2.  **图结构分析 (`CheckCellWidth`, `Get...Gates`)**: 遍历电路，建立`driver/reader`连接关系，并对所有组合逻辑门进行拓扑排序。
3.  **预计算 (`GenerateCuts`)**: 为每一个门，暴力枚举出所有可能的LUT覆盖方式（割集），作为“候选方案库”。
4.  **迭代优化 (主循环)**:
    *   **前向遍历 (`TraverseFWD`)**: 从输入到输出，根据当前的`depth`和`af`成本，为每个节点**临时选择**一个最优的覆盖方案。
    *   **后向遍历 (`TraverseBWD`)**: 从输出到输入，根据前向遍历的临时选择，**修正**每个节点的`height`和扇出信息，为下一轮迭代提供更精确的成本评估数据。
    *   (重复以上两步 `MAX_INTERATIONS` 次)
5.  **最终实现 (`ConeToLUTs`)**: 在所有迭代结束后，根据最后一轮选出的最优方案 (`best_bit2cut`)，为每个要实现的逻辑锥**实例化**一个对应的`GTP_LUT`，并计算其`INIT`参数。
6.  **清理**: 删除所有旧的、已经被覆盖掉的通用逻辑门。
7.  **完成**: 内存中的电路已经从通用逻辑门网表，完全转换为了Pango LUT网表。