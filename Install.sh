sudo apt-get install build-essential
sudo apt-get install git-core
sudo apt-get install cmake
sudo apt-get install doxygen
sudo apt-get install libusb-1.0-devel
sudo apt-get install libconfuse-dev 
sudo apt-get install swig python-dev 
sudo apt-get install libboost-all-dev 
mkdir libftdi
cd libftdi
git clone git://developer.intra2net.com/libftdi
cd libftdi
mkdir build
cd build
cmake ..
make
sudo make install
sudo apt-get install build-essential gcc cmake git
sudo apt-get install cdbs libcfitsio-dev libnova-dev libusb-1.0-0-dev libjpeg-dev libusb-dev libtiff5-dev libftdi-dev fxload libkrb5-dev libcurl4-gnutls-dev libraw-dev libgphoto2-dev libgsl0-dev dkms libboost-regex-dev libgps-dev libdc1394-22-dev
cd
git clone https://github.com/iMhack/indi_cam90_ccd.git
cd indi_cam90_ccd
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr ..
make
sudo cp ../99-cam90.rules /etc/udev/rules.d/
sudo service udev restart
echo indiserver -v -m 100 ./indi_cam90_ccd
sudo make install
