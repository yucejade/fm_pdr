export VCPKG_ROOT=/home/kevin/Dev/tools/vcpkg
rm -rf build
cmake -S . -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_TOOLCHAIN_FILE=${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake -DCMAKE_BUILD_TYPE=debug -DCMAKE_INSTALL_PREFIX=build/package -DBUILD_WITH_THIRDPARTY=ON -Wnarrowing
#cmake -S . -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=debug -DCMAKE_INSTALL_PREFIX=build/package -DBUILD_WITH_THIRDPARTY=ON -Wnarrowing
cmake --build build -j10 --target install
