# ACG_Project

This is the ACG final project by Xianbang Wang and Qicheng Xu.

## Demo

https://github.com/user-attachments/assets/7db2e298-07b2-494e-8f70-4918b82ab11a


## Requirements

- To use `SPLASH_SURF` method in `renderFluid()`, one must install **splashsurf**:

```
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
cargo install splashsurf
```

- To enable rendering using Blender, please first install the following packages:
```py
pip install bpy trimesh pyyaml
brew install blender # or directly download blender from official cite https://www.blender.org/.
```
and make sure that `src/render.py` can run successfully. Then, change the configuration file under `./config`:
```
blender:
  enabled: true
```

Plus, make sure you install the packages into the blender's python environment. If you work on linux, I recommend you to download the blender from the official cite instead of using `apt-get`.