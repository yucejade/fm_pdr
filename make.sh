#!/bin/bash

# 定义清理函数
clean() {
    if [ -d "build" ]; then
        echo "Removing build directory..."
        rm -rf build/
        echo "Clean completed."
    else
        echo "build directory does not exist, nothing to clean."
    fi
}

# 定义编译第三方库函数
build_thirdparty() {
    if [ -d "thirdparty" ]; then
        echo "Building thirdparty libraries..."

        # 创建package的基本目录结构
        mkdir -p ./build/package/bin/
        mkdir -p ./build/package/lib/
        mkdir -p ./build/package/include/

        # 进入第三方库目录并编译
        # 编译openblas库
        cd thirdparty/OpenBLAS-develop || exit 1
        cmake -B ../../build/thirdparty/OpenBLAS-develop -DBUILD_STATIC_LIBS=ON -DBUILD_SHARED_LIBS=OFF -DCMAKE_INSTALL_PREFIX=../../build/package -S .
        make -C ../../build/thirdparty/OpenBLAS-develop install
        cd ../.. || exit 1

        # 编译Eigen库
        cd thirdparty/eigen-3.4.1 || exit 1
        cmake -B ../../build/thirdparty/eigen-3.4.1 -DCMAKE_INSTALL_PREFIX=../../build/package -S .
        make -C ../../build/thirdparty/eigen-3.4.1 install
        cd ../.. || exit 1

        # 编译Fusion-main库
        cd thirdparty/Fusion-main || exit 1
        cmake -B ../../build/thirdparty/Fusion-main -DCMAKE_INSTALL_PREFIX=../../build/package -S .
        make -C ../../build/thirdparty/Fusion-main
        cp -r ../../build/thirdparty/Fusion-main/Fusion/libFusion.a ../../build/package/lib/
        mkdir -p ../../build/package/include/Fusion/
        cp -r Fusion/*.h ../../build/package/include/Fusion/
        cd ../.. || exit 1

        # 编译iir1-master库
        cd thirdparty/iir1-master || exit 1
        cmake -B ../../build/thirdparty/iir1-master -DCMAKE_INSTALL_PREFIX=../../build/package -S .
        make -C ../../build/thirdparty/iir1-master install
        cd ../.. || exit 1

        # 编译geographiclib-main库
        cd thirdparty/geographiclib-main || exit 1
        cmake -B ../../build/thirdparty/geographiclib-main -DBUILD_SHARED_LIBS=OFF -DCMAKE_INSTALL_PREFIX=../../build/package -S .
        make -C ../../build/thirdparty/geographiclib-main install
        cd ../.. || exit 1

        # 编译dlib-20.0库
        cd thirdparty/dlib-20.0 || exit 1
        cmake -B ../../build/thirdparty/dlib-20.0 -DCMAKE_INSTALL_PREFIX=../../build/package -S .
        make -C ../../build/thirdparty/dlib-20.0 install
        cd ../.. || exit 1

        # 编译rapidcsv-master库
        cd thirdparty/rapidcsv-master || exit 1
        cmake -B ../../build/thirdparty/rapidcsv-master -DCMAKE_INSTALL_PREFIX=../../build/package -S .
        make -C ../../build/thirdparty/rapidcsv-master install
        cd ../.. || exit 1

        # 编译rapidjson-master库
        cd thirdparty/rapidjson-master || exit 1
        cmake -B ../../build/thirdparty/rapidjson-master -DCMAKE_INSTALL_PREFIX=../../build/package -S .
        make -C ../../build/thirdparty/rapidjson-master install
        cd ../.. || exit 1

        echo "Thirdparty build completed."
    else
        echo "thirdparty directory not found, build failed."
        exit 1
    fi
}

# 定义编译PDR项目函数
build_pdr() {
    if [ -d "src" ]; then
        echo "Building PDR project..."
        # 创建build目录
        mkdir -p build/src
        # 进入src目录并编译（此处假设使用cmake，可根据实际情况修改）
        cd src || exit 1
        cmake -B ../build/src -DCMAKE_INSTALL_PREFIX=../build/package -S .
        make -C ../build/src install
        cd .. || exit 1
        echo "PDR build completed."
    else
        echo "src directory not found, build failed."
        exit 1
    fi
}

# 定义编译PDRTest项目函数
build_test() {
    if [ -d "example" ]; then
        echo "Building PDRTest project..."
        # 创建build目录
        mkdir -p build/example
        # 进入example目录并编译（此处假设使用cmake，可根据实际情况修改）
        cd example || exit 1
        cmake -B ../build/example -DCMAKE_INSTALL_PREFIX=../build/package -S .
        make -C ../build/example install
        cd ../.. || exit 1
        echo "PDRTest build completed."
    else
        echo "src directory not found, build failed."
        exit 1
    fi
}

# 参数检查
if [ $# -ne 1 ]; then
    echo "Usage: $0 [clean|thirdparty|pdr|test|build|rebuild]"
    exit 1
fi

case "$1" in
    clean)
        clean
        ;;
    thirdparty)
        build_thirdparty
        ;;
    pdr)
        build_pdr
        ;;
    test)
        build_test
        ;;
    build)
        build_thirdparty
        build_pdr
        build_test
        ;;
    rebuild)
        clean
        build_thirdparty
        build_pdr
        build_test
        ;;
    *)
        echo "Invalid argument: $1"
        echo "Usage: $0 [clean|thirdparty|pdr|test|build|rebuild]"
        exit 1
        ;;
esac