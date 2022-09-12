## Create directory to setup Trajnet++
mkdir trajnet++
cd trajnet++ 

## Clone Repositories
git clone https://github.com/vita-epfl/trajnetplusplusdataset.git
git clone https://github.com/vita-epfl/trajnetplusplusbaselines.git

## Download Requirements
cd trajnetplusplusbaselines/ 
pip install -e .

cd ../trajnetplusplusdataset/ 
pip install -e .
pip install -e '.[test, plot]'

## Download Repository
wget https://github.com/sybrenstuvel/Python-RVO2/archive/master.zip
unzip master.zip
rm master.zip

## Setting up ORCA (steps provided in the Python-RVO2 repo)
cd Python-RVO2-master/
pip install cmake
pip install cython
python setup.py build
python setup.py install
cd ../

## Download Repository
wget https://github.com/svenkreiss/socialforce/archive/refs/heads/main.zip
unzip main.zip
rm main.zip

## Setting up Social Force
cd socialforce-main/
pip install -e .
cd ../
