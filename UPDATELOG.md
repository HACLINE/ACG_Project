### 2024.12.19 15:00
by xqc
Done:
- Fluid load from mesh.

### 2024.12.14 14:00
by wxb
Done:
- `render.py` rewritten.
- Wetting effect of cloth when pouring fluid on it. Also added this feature in `collision.cu` to be compatible with CUDA.

### 2024.11.26 18:00
by xqc & wxb
Done:
- **MIDTERM REPORT**.

### 2024.11.19 2:00
by xqc
Done:
- Finish CUDA on `PICFLIPFluid`, `XPBDCloth` and coupling of both. Really annoying but finally done.

TODO:
- Maybe the file storage is the bottleneck. Change to subprocessing? Plus, consider reconstruct surface after simulation. It takes space and time to reconstruct and store the surface.

### 2024.11.17 18:00
by xqc
Done:
- Store data and render later with blender. 
- Allow disable GL rendering during simulation. (May be useful for speedup)
  
TODO:
- CUDA
- Parameter tuning

### 2024.11.16 13:30
by wxb
Done:
- Render fluid-cloth coupling using `blender`. Implemented with python: see `src/render.py`.

### 2024.10.30 15:00
by wxb
Done:
- Cloth-fluid coupling 1.0 (collision check using spatial hash; collision update without friction).

TODO:
- Have permeation problem, need to be fixed;
- fluid looks bad after collide with cloth and fall to the ground. Need to check the reason.
- Other coupling (friction) or other coupling method can be implemented.

### 2024.10.29 10:30
by wxb
Done: 
- Four method to render fluid: `POINT_CLOUD`, `VOXEL`, `MARCHING_CUBES`, `SPLASH_SURF` (default, using pack `splashsurf`). The first DEMO of mesh-like fluid simulation!

TODO:
- Still have strange scenes;
- Cloth-fluid coupling.

### 2024.10.28 20:30
by wxb
Done: 
- Reconstruct fluid surface using marching cube method implemented by C++.

TODO:
- Still have artifacts. Need to debug, or implement using some libraries.

### 2024.10.28 16:00
by xqc
Done: Bug fixed. 

### 2024.10.25 21:00
by xqc
Done:
- Apply a Lagrangian-Eulerian fluid simulation. Notice that a discount factor on velocity is applied to avoid the instability of the simulation.

TODO:
- Coupling.

### 2024.10.24 11:00
by wxb
Done:
- Cloth 1.6: Collision with Triangle & Sphere.
- Change `damping_` and `update()` of `class Cloth`.

TODO:
- Adjust Fluid and then do Cloth-Fluid interaction.

### 2024.10.19 22:00
by wxb
Done:
- Cloth: self-collision check
- Cloth: collision with environment (wall)

TODO:
- Still have artifacts. Need debug.

### 2024.10.18 11:00
by wxb
Done:
- Basic geometry of CLOTH: mass-spring model
- Cloth simulation

Note:
- Head file `GL/glut.h` is changed into `GLUT/glut.h`.

TODO:
- Collision check with other objects

### 2024.10.06 2:00
by xqc
Done:
- Parallelization with CUDA, great speedup

TODO:
- Seems still some problem with fluid simulation. Maybe because of the interaction between fluid and box, which is written by me with reference to the rigidbody collision. Need debug but not today.

#### 2024.10.04 23:00
by xqc
Done:
- [DFSPH](https://dl.acm.org/doi/abs/10.1145/2786784.2786796) fluid simulation (seems done, may debug after parallelization implemented)

TODO:
- Parallelization
- Collision with rigidbody
- Perhaps changes rigidbody to particles and deal with them together?

#### 2024.10.03 20:00
by xqc
Done:
- Small modify on code structure. Now more convenient to implement different methods.

#### 2024.10.03 15:00
by xqc
Done:
- Rigid body collision with box, based on impulse method([Reference](https://www.aliyundrive.com/s/YGuzfDCzw4n/folder/61824d985307bbf3920044b4afd48abb633441f6))

TODO:
- Make it better
- Move on

#### 2024.10.01 12:00
by wxb
Done:
- Construct whole pipeline in main.cpp (make a video by combining frames in ./figures)
- Add basic gravity forces on rigid bodies

#### 2024.10.01 00:00
by xqc
Done:
- Reconstruct render, now with more clear structure
- Define basic classes `Rigidbody` and `Fluid`
- Render properly
- Load models from file
- Write yaml config file

TODO:
- Augment two classes and config
- Implement basic physics

#### 2024.09.30 17:00
by wxb
Done:
- Use OpenGL to render the scene
- Basic structure