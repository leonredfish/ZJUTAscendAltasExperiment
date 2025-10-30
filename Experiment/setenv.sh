#!/bin/bash

script_name=$0

if [ "$(id -u)" -ne 0 ]; then
    echo "Please run this script with root user."
    exit 1
fi

if [ ! -e /usr/lib64/libmedia_mini.so ];then
    cp software/requirement/libmedia_mini.so /usr/lib64
    ldconfig
fi

if [ ! -d /usr/include/opencv4 ];then
    dpkg -i software/requirement/opencv/*deb
fi

if [ ! -e /etc/ld.so.conf.d/ascend_ddk.conf ];then
    touch /etc/ld.so.conf.d/ascend_ddk.conf
    echo "$(pwd)/software/ascend_ddk/arm/lib/" >> /etc/ld.so.conf.d/ascend_ddk.conf
    ldconfig
fi

mv /etc/netplan/01-netcfg.yaml /etc/netplan/01-netcfg.yaml.back
touch /etc/netplan/01-netcfg.yaml
echo "network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: no
      addresses: [192.168.0.2/24]
      nameservers:
        addresses: [8.8.8.8]
        addresses: [114.114.114.114]

    usb0:
      dhcp4: no
      addresses: [192.168.1.2/24]
" > /etc/netplan/01-netcfg.yaml

if [ -z $(groups HwHiAiUser | grep video) ];then
  usermod -aG video HwHiAiUser
fi

sed -i 's/192\.168\.137\.100/192.168.1.2/g' ../samples/notebooks/start_notebook.sh

cp $(pwd)/software/helloworld /usr/bin
cp $(pwd)/software/atlas200dk-init.service /etc/systemd/system/
cp $(pwd)/software/atlas200dk-init.sh /usr/local/bin/
chmod 744 /usr/local/bin/atlas200dk-init.sh
chmod 644 /etc/systemd/system/atlas200dk-init.service
systemctl daemon-reload
systemctl enable atlas200dk-init.service
systemctl start atlas200dk-init.service

dd if=software/dt.img of=/dev/mmcblk1 count=4096 seek=114688 bs=512
reboot

