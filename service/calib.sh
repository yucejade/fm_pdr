#! /bin/bash

echo "开始校准倒计时:3"
sleep 1
echo "开始校准倒计时:2"
sleep 1
echo "开始校准倒计时:1"
sleep 1
echo "开始校准!!!"

export PACKAGE_PATH=/home/kevin/fm_pdr/build/package
rm ${PACKAGE_PATH}/bin/mag_calib_data -fr
sudo systemctl start mag_data.service

sleep 57 
echo "结束校准倒计时:3"
sleep 1
echo "结束校准倒计时:2"
sleep 1
echo "结束校准倒计时:1"
sleep 1
echo "结束校准!!!"

sudo systemctl stop mag_data.service
${PACKAGE_PATH}/bin/mag_calib4 -t 1 -d ${PACKAGE_PATH}/bin/mag_calib_data/Magnetometer.csv -o ${PACKAGE_PATH}/bin/mag_calib_data/mag_calibration.csv
cp ${PACKAGE_PATH}/bin/mag_calib_data/mag_calibration.csv ${PACKAGE_PATH}/conf/mag_calibration.csv
