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