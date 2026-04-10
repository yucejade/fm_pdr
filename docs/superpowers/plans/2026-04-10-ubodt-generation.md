# UBODT Generation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add UBODT generation capability to PDRTest program, allowing users to generate UBODT files from road network shapefiles without external tools.

**Architecture:** Add `--network`, `--ubodt`, `--delta` command-line arguments to `example/main.cpp`. When `--network` and `--ubodt` are provided, load the shapefile via FMM's `Network` class, build a `NetworkGraph`, and call `UBODTGenAlgorithm::generate_ubodt()` to produce the output file. Update `example/CMakeLists.txt` to link FMMLIB.

**Tech Stack:** C++17, FMM library (Network, NetworkGraph, UBODTGenAlgorithm), GDAL/OGR (via FMM for shapefile reading)

---

## File Structure

| File | Action | Responsibility |
|------|--------|----------------|
| `example/CMakeLists.txt` | Modify | Add FMM include path and link FMMLIB |
| `example/main.cpp` | Modify | Add UBODT generation parameters and logic |

---

### Task 1: Add FMM Dependencies to CMakeLists.txt

**Files:**
- Modify: `example/CMakeLists.txt`

- [ ] **Step 1: Add fmm include path and FMMLIB link**

In `example/CMakeLists.txt`, add `${CMAKE_INSTALL_PREFIX}/include/fmm` to include directories, and add `FMMLIB` to the link libraries.

Current `INCLUDE_DIRECTORIES` block (lines 20-26):
```cmake
INCLUDE_DIRECTORIES(${PROJECT_NAME}
    PRIVATE
    ${CMAKE_INSTALL_PREFIX}/include
    ${CMAKE_INSTALL_PREFIX}/include/FmPDR
    ${CMAKE_INSTALL_PREFIX}/include/Fusion
    ${CMAKE_INSTALL_PREFIX}/include/eigen3
    )
```

Change to:
```cmake
INCLUDE_DIRECTORIES(${PROJECT_NAME}
    PRIVATE
    ${CMAKE_INSTALL_PREFIX}/include
    ${CMAKE_INSTALL_PREFIX}/include/FmPDR
    ${CMAKE_INSTALL_PREFIX}/include/Fusion
    ${CMAKE_INSTALL_PREFIX}/include/eigen3
    ${CMAKE_INSTALL_PREFIX}/include/fmm
    )
```

Current `TARGET_LINK_LIBRARIES` (line 33):
```cmake
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC FmPDR iir_static dlib openblas Fusion GeographicLib)
```

Change to:
```cmake
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC FmPDR iir_static dlib openblas Fusion GeographicLib FMMLIB)
```

- [ ] **Step 2: Verify CMakeLists.txt compiles**

Run: `./make.sh build 2>&1 | tail -20`
Expected: No errors related to FMMLIB or include paths. Should see `PDR build completed`.

---

### Task 2: Add UBODT Generation Parameters and Logic to main.cpp

**Files:**
- Modify: `example/main.cpp`

- [ ] **Step 1: Add FMM includes**

Add the following includes after the existing includes (after line 13, `#include <vector>`):

```cpp
#include "network/network.hpp"
#include "network/network_graph.hpp"
#include "mm/fmm/ubodt_gen_algorithm.hpp"
```

- [ ] **Step 2: Add new options to ArgParser help text**

Replace the `showHelp()` method body (lines 48-56) with:

```cpp
    void showHelp() const
    {
        std::cout << "Usage: pdr [options]\n"
                  << "Options:\n"
                  << "  -e, --evaluation\t\t\t是否打印指标评估\n"
                  << "  -c, --config <配置文件路径>\t\t指定PDR配置文件路径，默认使用../conf/config.json\n"
                  << "  -t, --train <样本数据路径>\t\t表示需要加载的<样本数据路径>，训练模型输出到model_file_name配置项设置的路径下\n"
                  << "  -d, --dataset <PDR数据路径>\t\t表示需要加载的<PDR测试数据路径>，使用model_file_name配置项设置路径下的模型文件进行推算\n"
                  << "  --network <路网shapefile路径>\t\t输入路网shapefile文件，用于生成UBODT\n"
                  << "  --ubodt <输出文件路径>\t\t\t输出UBODT文件路径\n"
                  << "  --delta <数值>\t\t\tUBODT上界参数（默认0.02，单位与路网坐标系一致）\n"
                  << "  -h, --help\t\t\t\t帮助信息\n";
    }
```

- [ ] **Step 3: Add UBODT parameter parsing in main()**

After the existing parameter parsing block (after line 84, where `pdr_config_path` is assigned), add:

```cpp
    std::string network_path = parser.getOption( "--network" );
    std::string ubodt_path = parser.getOption( "--ubodt" );
    std::string delta_str = parser.getOption( "--delta" );
    double delta_value = 0.02;  // 默认值，WGS84下约2.2km
    if ( ! delta_str.empty() )
        delta_value = std::stod( delta_str );
```

- [ ] **Step 4: Update parameter validation**

Change the validation block (lines 87-92) from:

```cpp
    // 验证必要参数
    if ( train_dataset_path.empty() && pdr_dataset_path.empty() )
    {
        std::cerr << "Argument error.\n";
        parser.showHelp();
        return -1;
    }
```

To:

```cpp
    bool ubodt_mode = ! network_path.empty() && ! ubodt_path.empty();

    // 验证必要参数
    if ( train_dataset_path.empty() && pdr_dataset_path.empty() && ! ubodt_mode )
    {
        std::cerr << "Argument error.\n";
        parser.showHelp();
        return -1;
    }
```

- [ ] **Step 5: Add UBODT generation logic before the try block**

Insert the UBODT generation logic before the existing `try` block (before line 101). This keeps it separate from the PDR logic:

```cpp
    // UBODT 生成模式
    if ( ubodt_mode )
    {
        try
        {
            std::cout << "Loading network from " << network_path << std::endl;
            FMM::NETWORK::Network network( network_path );
            std::cout << "Network loaded: " << network.get_node_count() << " nodes, "
                      << network.get_edge_count() << " edges" << std::endl;

            FMM::NETWORK::NetworkGraph graph( network );
            FMM::MM::UBODTGenAlgorithm ubodt_gen( network, graph );

            std::cout << "Generating UBODT with delta=" << delta_value << " ..." << std::endl;
            std::string result = ubodt_gen.generate_ubodt( ubodt_path, delta_value, false, true );
            std::cout << result << std::endl;
            std::cout << "UBODT saved to " << ubodt_path << std::endl;
            return 0;
        }
        catch ( const std::exception& e )
        {
            std::cerr << "Error: " << e.what() << std::endl;
            return -1;
        }
    }
```

- [ ] **Step 6: Build and verify compilation**

Run: `./make.sh build 2>&1 | tail -20`
Expected: No compilation errors. Should see `PDR build completed`.

Run: `./build/package/bin/PDRTest --help`
Expected: Help text includes `--network`, `--ubodt`, `--delta` options.

- [ ] **Step 7: Commit**

```bash
git add example/main.cpp example/CMakeLists.txt
git commit -m "feat: add UBODT generation to PDRTest with --network/--ubodt/--delta options"
```

---

## Self-Review

**1. Spec coverage:**
- `--network <shapefile>` input: Covered in Task 2 Steps 2-3
- `--ubodt <output>` output: Covered in Task 2 Steps 2-3
- `--delta <value>` parameter with default 0.02: Covered in Task 2 Step 3
- UBODT generation using FMM API: Covered in Task 2 Step 5
- CMakeLists.txt update for FMM dependency: Covered in Task 1

**2. Placeholder scan:** No TBD/TODO found. All steps contain complete code.

**3. Type consistency:** `network_path` is `std::string`, `ubodt_path` is `std::string`, `delta_value` is `double` — consistent with `Network(filename)`, `generate_ubodt(filename, delta)`.
