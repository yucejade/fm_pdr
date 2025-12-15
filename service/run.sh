#! /bin/bash

echo "开始PDR倒计时:3"
sleep 1
echo "开始PDR倒计时:2"
sleep 1
echo "开始PDR倒计时:1"
sleep 1
echo "开始PDR!!!"

export PACKAGE_PATH=/home/kevin/Dev/yuce/fm_pdr/build/package
rm ${PACKAGE_PATH}/bin/output_sensor_data ${PACKAGE_PATH}/bin/Trajectory.csv ${PACKAGE_PATH}/bin/Trajectory_bak.csv -fr
sudo systemctl start pdr_test.service

echo -e "\n服务已启动，按q停止服务..."
while true; do
  read -n1 -s input  # 读取单个字符，不回显
  if [[ $input == "q" ]]; then
    echo -e "\n正在停止服务..."
    sudo systemctl stop pdr_test.service
    echo "服务已停止"
    mv ${PACKAGE_PATH}/bin/Trajectory.csv ${PACKAGE_PATH}/bin/Trajectory_bak.csv
    ${PACKAGE_PATH}/bin/PDRTest -d ${PACKAGE_PATH}/bin/output_sensor_data -o ${PACKAGE_PATH}/bin/Trajectory.csv -e
    exit 0
  fi
done
