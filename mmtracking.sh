#install dependency
pip install openmim
mim install mmcv-full
mim install mmdet

#install mmtracking
git clone https://github.com/open-mmlab/mmtracking.git
cd mmtracking
pip install -r requirements/build.txt
pip install -v -e .  # or "python setup.py develop"
cd ..