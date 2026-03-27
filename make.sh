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
        PACKAGE_PATH=${PWD}/build/package
        BUILD_TRIPLET=$(gcc -dumpmachine)
        export LD_LIBRARY_PATH=${PACKAGE_PATH}/lib
        export PKG_CONFIG_PATH=${PACKAGE_PATH}/lib/pkgconfig:$PKG_CONFIG_PATH

        # 进入第三方库目录并编译
        # 编译openblas库
        cd thirdparty/OpenBLAS-develop || exit 1
        cmake -B ../../build/thirdparty/OpenBLAS-develop -DBUILD_TESTING=OFF -DNOFORTRAN=ON -DBUILD_STATIC_LIBS=ON -DBUILD_SHARED_LIBS=OFF -DCMAKE_INSTALL_PREFIX=../../build/package -S .
        sudo make -C ../../build/thirdparty/OpenBLAS-develop install
        cd ../.. || exit 1

        # 编译Eigen库
        cd thirdparty/eigen-3.4.1 || exit 1
        cmake -B ../../build/thirdparty/eigen-3.4.1 -DCMAKE_INSTALL_PREFIX=../../build/package -S .
        sudo make -C ../../build/thirdparty/eigen-3.4.1 install
        cd ../.. || exit 1

        # 编译Fusion-main库
        cd thirdparty/Fusion-main || exit 1
        cmake -B ../../build/thirdparty/Fusion-main -DCMAKE_INSTALL_PREFIX=../../build/package -S .
        sudo make -C ../../build/thirdparty/Fusion-main
        cp -r ../../build/thirdparty/Fusion-main/Fusion/libFusion.a ../../build/package/lib/
        mkdir -p ../../build/package/include/Fusion/
        cp -r Fusion/*.h ../../build/package/include/Fusion/
        cd ../.. || exit 1

        # 编译iir1-master库
        cd thirdparty/iir1-master || exit 1
        cmake -B ../../build/thirdparty/iir1-master -DCMAKE_INSTALL_PREFIX=../../build/package -S .
        sudo make -C ../../build/thirdparty/iir1-master install
        cd ../.. || exit 1

        # 编译geographiclib-main库
        cd thirdparty/geographiclib-main || exit 1
        cmake -B ../../build/thirdparty/geographiclib-main -DBUILD_SHARED_LIBS=OFF -DCMAKE_INSTALL_PREFIX=../../build/package -S .
        sudo make -C ../../build/thirdparty/geographiclib-main install
        cd ../.. || exit 1

        # 编译dlib-20.0库
        cd thirdparty/dlib-20.0 || exit 1
        cmake -B ../../build/thirdparty/dlib-20.0 -DCMAKE_INSTALL_PREFIX=../../build/package -S .
        sudo make -C ../../build/thirdparty/dlib-20.0 install
        cd ../.. || exit 1

        # 编译rapidcsv-master库
        cd thirdparty/rapidcsv-master || exit 1
        cmake -B ../../build/thirdparty/rapidcsv-master -DCMAKE_INSTALL_PREFIX=../../build/package -S .
        sudo make -C ../../build/thirdparty/rapidcsv-master install
        cd ../.. || exit 1

        # 编译rapidjson-master库
        cd thirdparty/rapidjson-master || exit 1
        cmake -B ../../build/thirdparty/rapidjson-master -DCMAKE_INSTALL_PREFIX=../../build/package -S .
        sudo make -C ../../build/thirdparty/rapidjson-master install
        cd ../.. || exit 1

        # 编译libgpiod-master库
        mkdir -p build/thirdparty/libgpiod-master && cd build/thirdparty/libgpiod-master || exit 1
        ../../../thirdparty/libgpiod-master/autogen.sh
        ../../../thirdparty/libgpiod-master/configure --enable-shared=no --prefix=$(pwd)/../../package
        sudo make install
        cd ../../.. || exit 1

        # 编译moodycamel库
        cd thirdparty/concurrentqueue-master || exit 1
        cmake -B ../../build/thirdparty/concurrentqueue-master -DCMAKE_INSTALL_PREFIX=../../build/package -S .
        sudo make -C ../../build/thirdparty/concurrentqueue-master install
        cd ../.. || exit 1

       ### 编译fmm依赖库
       # 编译ncurses库
       mkdir -p build/thirdparty/fmm//ncurses-6.6 && cd build/thirdparty/fmm//ncurses-6.6 || exit 1
       ../../../../thirdparty/fmm//ncurses-6.6/configure --with-shared --disable-widec --prefix=$(pwd)/../../../package --build=${BUILD_TRIPLET}
       sudo make install
       cd ../../../.. || exit 1

       # 编译readline库
       mkdir -p build/thirdparty/fmm/readline && cd build/thirdparty/fmm/readline || exit 1
       export bash_cv_termcap_lib=ncurses
       ../../../../thirdparty/fmm/readline/configure --with-curses CPPFLAGS="-I${PACKAGE_PATH}/include" CFLAGS="-I${PACKAGE_PATH}/include" LDFLAGS="-L${PACKAGE_PATH}/lib" SHLIB_LIBS="-lncurses" --prefix=$(pwd)/../../../package --build=${BUILD_TRIPLET}
       make install
       cd ../../../.. || exit 1

        # 编译sqlite库
        mkdir -p build/thirdparty/fmm/sqlite-src-3510300 && cd build/thirdparty/fmm/sqlite-src-3510300 || exit 1
        ../../../../thirdparty/fmm/sqlite-src-3510300/configure CFLAGS="-DSQLITE_ENABLE_RTREE=1" LDFLAGS="-L${PACKAGE_PATH}/lib -lreadline -lncurses -lz -lm" --prefix=$(pwd)/../../../package --build=${BUILD_TRIPLET}
        sudo make install
        cd ../../../.. || exit 1

        # 编译libminizip库
        cd thirdparty/fmm/libminizip-cmake || exit 1
        cmake -B ../../../build/thirdparty/fmm/libminizip-cmake -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_INSTALL_PREFIX=../../../build/package -S .
        sudo make -C ../../../build/thirdparty/fmm/libminizip-cmake install
        cd ../../.. || exit 1

        # 编译libtiff库
        cd thirdparty/fmm/libtiff-master || exit 1
        cmake -B ../../../build/thirdparty/fmm/libtiff-master -DCMAKE_INSTALL_PREFIX=../../../build/package -S .
        sudo make -C ../../../build/thirdparty/fmm/libtiff-master install
        cd ../../.. || exit 1

        # 编译geos库
        cd thirdparty/fmm/geos || exit 1
        cmake -B ../../../build/thirdparty/fmm/geos -DCMAKE_INSTALL_PREFIX=../../../build/package -S .
        sudo make -C ../../../build/thirdparty/fmm/geos install
        cd ../../.. || exit 1

        # 编译PROJ库
        cd thirdparty/fmm/PROJ || exit 1
        cmake -B ../../../build/thirdparty/fmm/PROJ -DEXE_SQLITE3=../../../build/package/bin/sqlite3 -DCMAKE_INSTALL_PREFIX=../../../build/package -DCMAKE_PREFIX_PATH=../../../build/package -S .
        sudo make -C ../../../build/thirdparty/fmm/PROJ install
        cd ../../.. || exit 1

#       # 编译libspatialite库
#       cd thirdparty/fmm/libspatialite-5.1.0 || exit 1
#       wget -O config.guess 'https://git.savannah.gnu.org/gitweb/?p=config.git;a=blob_plain;f=config.guess;hb=HEAD'
#       wget -O config.sub 'https://git.savannah.gnu.org/gitweb/?p=config.git;a=blob_plain;f=config.sub;hb=HEAD'
#       cd ../../.. || exit 1
#       mkdir -p build/thirdparty/fmm/libspatialite-5.1.0 && cd build/thirdparty/fmm/libspatialite-5.1.0 || exit 1
#       if [ -f "thirdparty/fmm/libspatialite-5.1.0/src/headers/spatialite/gaiaconfig.h" ]; then
#           rm thirdparty/fmm/libspatialite-5.1.0/src/headers/spatialite/gaiaconfig.h
#       fi
#       ../../../../thirdparty/fmm/libspatialite-5.1.0/configure --enable-freexl=no --enable-rttopo=no --with-geosconfig=${PACKAGE_PATH}/bin/geos-config CFLAGS="-I${PACKAGE_PATH}/include" LDFLAGS="-L${PACKAGE_PATH}/lib" LIBS="-lreadline -lncurses -lgeos_c -lgeos -lproj -lsqlite3 -ltiff -lstdc++ -lpthread -lm -ldl" --prefix=$(pwd)/../../../package --build=${BUILD_TRIPLET}
#       sudo make install
#       cd ../../../.. || exit 1

#       # 编译openssl库
#       mkdir -p build/thirdparty/fmm/openssl && cd build/thirdparty/fmm/openssl || exit 1
#       ARCH=$(uname -m)
#       case "$ARCH" in
#           x86_64)
#               OPENSSL_TARGET="linux-x86_64"
#               ;;
#           aarch64|arm64)
#               OPENSSL_TARGET="linux-aarch64"
#               ;;
#           *)
#               echo "不支持的架构: $ARCH"
#               exit 1
#               ;;
#       esac
#       ../../../../thirdparty/fmm/openssl/Configure ${OPENSSL_TARGET} --prefix=${PACKAGE_PATH} --openssldir=${PACKAGE_PATH} --libdir=lib shared zlib
#       sudo make install
#       cd ../../../.. || exit 1

#       # 编译curl库
#       cd thirdparty/fmm/curl || exit 1
#       cmake -B ../../../build/thirdparty/fmm/curl -DCURL_USE_OPENSSL=ON -DCMAKE_INSTALL_PREFIX=../../../build/package -DCMAKE_PREFIX_PATH=../../../build/package -DCURL_USE_LIBPSL=OFF -DUSE_LIBIDN2=OFF -DUSE_NGHTTP2=OFF -DCURL_USE_OPENSSL=ON -S .
#       sudo make -C ../../../build/thirdparty/fmm/curl install
#       cd ../../.. || exit 1

        # 编译gdal库
        cd thirdparty/fmm/gdal || exit 1
        cmake -B ../../../build/thirdparty/fmm/gdal -DCMAKE_INSTALL_PREFIX=../../../build/package -DCMAKE_PREFIX_PATH=../../../build/package -DGDAL_USE_CURL=OFF -DGDAL_USE_SPATIALITE=OFF -DGDAL_ENABLE_DRIVER_GRIB=OFF -DGDAL_USE_ZSTD=OFF -DPROJ_DIR=${PACKAGE_PATH}/lib/cmake/proj -DCMAKE_BUILD_TYPE=release -S .
        sudo make -C ../../../build/thirdparty/fmm/gdal install
        cd ../../.. || exit 1

        # 编译fmm库
        cd thirdparty/fmm/fmm-master || exit 1
        cmake -B ../../../build/thirdparty/fmm/fmm-master -DCMAKE_INSTALL_PREFIX=../../../build/package -DCMAKE_PREFIX_PATH=../../../build/package -DCMAKE_BUILD_TYPE=release -DFMM_INSTALL_HEADER=yes -S .
        sudo make -C ../../../build/thirdparty/fmm/fmm-master install
        cd ../../.. || exit 1
 
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
        # 进入src目录并编译
        cd src || exit 1
        cmake -B ../build/src -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=debug -DCMAKE_INSTALL_PREFIX=../build/package -S .
        sudo make -C ../build/src install
        cd .. || exit 1

        doxygen Doxyfile
        rm build/package/docs -fr && mv -f docs build/package

        echo "PDR build completed."
    else
        echo "src directory not found, build failed."
        exit 1
    fi
}

# 定义编译PDRTest项目函数
build_test() {
    if [ -d "example" ]; then
        echo "Building PDRTestFromFile project..."
        # 创建build目录
        mkdir -p build/example
        # 进入example目录并编译
        cd example || exit 1
        cmake -B ../build/example -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=debug -DCMAKE_INSTALL_PREFIX=../build/package -S .
        sudo make -C ../build/example install
        cd .. || exit 1
        echo "PDRTestFromFile build completed."

        echo "Building PDRTest project..."
        # 创建build目录
        mkdir -p build/example/plain_c
        # 进入example/plain_c目录并编译
        cd example/plain_c || exit 1
        cmake -B ../../build/example/plain_c -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=debug -DCMAKE_INSTALL_PREFIX=../../build/package -S .
        sudo make -C ../../build/example/plain_c install
        cd ../.. || exit 1
        echo "PDRTest build completed."

        echo "Building mag_calib project..."
        # 创建build目录
        mkdir -p build/example/mag_calib
        # 进入example/mag_calib
        cd example/mag_calib || exit 1
        cmake -B ../../build/example/mag_calib -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=debug -DCMAKE_INSTALL_PREFIX=../../build/package -S .
        sudo make -C ../../build/example/mag_calib install
        cd ../.. || exit 1
        echo "mag_calib build completed."

        echo "Building mag_calib2 project..."
        # 创建build目录
        mkdir -p build/example/mag_calib2
        # 进入example/mag_calib2
        cd example/mag_calib2 || exit 1
        cmake -B ../../build/example/mag_calib2 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=debug -DCMAKE_INSTALL_PREFIX=../../build/package -S .
        sudo make -C ../../build/example/mag_calib2 install
        cd ../.. || exit 1
        echo "mag_calib2 build completed."

        echo "Building mag_calib3 project..."
        # 创建build目录
        mkdir -p build/example/mag_calib3
        # 进入example/mag_calib3
        cd example/mag_calib3 || exit 1
        cmake -B ../../build/example/mag_calib3 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=debug -DCMAKE_INSTALL_PREFIX=../../build/package -S .
        sudo make -C ../../build/example/mag_calib3 install
        cd ../.. || exit 1
        echo "mag_calib3 build completed."

        echo "Building mag_calib4 project..."
        # 创建build目录
        mkdir -p build/example/mag_calib4
        # 进入example/mag_calib4
        cd example/mag_calib4 || exit 1
        cmake -B ../../build/example/mag_calib4 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=debug -DCMAKE_INSTALL_PREFIX=../../build/package -S .
        sudo make -C ../../build/example/mag_calib4 install
        cd ../.. || exit 1
        echo "mag_calib4 build completed."
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
