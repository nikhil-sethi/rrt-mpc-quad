pip3 install --upgrade pip
git submodule update --init --recursive
cd gym-pybullet-drones

# some mods for our particular project
sed -i 's/"1.9"/"^1.9"/g' pyproject.toml
sed -i '/torch/d' pyproject.toml
sed -i '/stable-baselines/d' pyproject.toml
sed -i '/tensorboard/d' pyproject.toml

# libraries
pip3 install cvxopt

# install gym-pybullet
pip3 install -e .
cd ..

