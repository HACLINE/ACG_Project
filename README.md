[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/cPlbGtcU)

# ACG_Project: Customized Cloth-Fluid Simulation Using C++

This is the ACG final project by Xianbang Wang and Qicheng Xu.

## Requirements

- To use `SPLASH_SURF` method in `renderFluid()`, one must install **splashsurf**:

```
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
cargo install splashsurf
```

- To enable rendering using Blender, please first install the following packages:
```py
pip install bpy trimesh pyyaml pandas
```
and make sure that `src/render.py` can run successfully. Then, change the configuration file under `./config`:
```
blender:
  enabled: true
```

Plus, make sure you install the packages into the blender's python environment. If you work on linux, I recommend you to download the blender from the official cite instead of using `apt-get`.

## Quick Start

Since this simulation is customized, you can design your own configuration in `config/` by writing a new `[NAME].yaml` file. You can also directly use the configuration written by us, which the hyperparameters are well tuned.

For instance, if you use `test.yaml` as the configuration, you should first run CMAKE. We have already provided a bash file for you. Run the following:
```
mkdir build && cd build
cmake ..
cd .. && sh compile.sh
```
Then, run `main.cpp` using the corresponding `.yaml` configuration:
```
./build/main --config test.yaml
```

## Acknowledgement

We appreciate [@Lyy-iiis (Yiyang Lu)](https://github.com/Lyy-iiis) for introducing splashsurf and Blender to us and inspiring some code in `src/render.py`!
