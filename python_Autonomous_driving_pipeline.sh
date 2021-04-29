sudo apt update
sudo apt install build-essential zlib1g-dev libncurses5-dev libgdbm-dev libnss3-dev libssl-dev libsqlite3-dev libreadline-dev libffi-dev wget libbz2-dev
wget https://www.python.org/ftp/python/3.7.4/Python-3.7.4.tgz
tar -xf Python-3.7.4.tgz
cd Python-3.7.4
./configure --enable-optimizations
make -j 8
sudo make altinstall
