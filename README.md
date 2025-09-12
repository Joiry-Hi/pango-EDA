# pango-EDA
EDA挑战赛

clone下来后修改文件夹名为pango，直接替换pango文件夹

之后在yosys根目录打开命令行终端（make clean可选）输入make -j(nproc)重新编译yosys，之后输入sudo make install进行安装

基础题代码在stitcher_pango.cc中，进阶题代码在mapper_pango.cc中

基础题命令为“stitcher”，和“mapper”一样用法，但作用对象是未合并的LUT网表，mapper则直接作用于门级网表

样例文件夹中有run_stitcher_verify.ys等已经写好的脚本，可直接利用yosys -s xxx.ys运行
