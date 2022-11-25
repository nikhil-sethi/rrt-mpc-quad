pip3 install --upgrade pip
cd ../gym-pybullet-drones/
sed -i 's/"1.9"/"^1.9"/g' pyproject.toml
pip3 install -e .
cd ../tooling