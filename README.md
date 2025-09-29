## Gate-level Netlist Buffering Demo

### Overview
This project perform buffer insertion to improve timing. The buffering algorithm is based on van Ginneken's dynamic programming based buffer insertion algorithm. 

### Build
```bash
git submodule update --init --recursive
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -j
```

### Usage
```bash
./opt -lib <liberty.lib> [-o <output.v>] <verilog.v>
```
- **-lib <liberty.lib>**: required Liberty file path.
- **-o <output.v>**: optional output file. Defaults to `optimized.v`.
- **<verilog.v>**: the first non-dashed argument; input Verilog netlist.

Examples:
```bash
# Using default output (optimized.v)
./build/opt -lib ./NangateOpenCellLibrary_typical.lib netlist.v -o buffered.v
```

