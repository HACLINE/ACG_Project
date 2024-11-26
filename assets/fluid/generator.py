
mass = 6e-3
radius = 0.01
filename = "two_cuboid2.fluid"
PRECISION = 6

particles = []

def generate_cuboid(center: list, scale: list, num: list):
    left_bottom = [center[0] - scale[0] / 2, center[1] - scale[1] / 2, center[2] - scale[2] / 2]
    step = [scale[0] / num[0], scale[1] / num[1], scale[2] / num[2]]
    for i in range(num[0]):
        for j in range(num[1]):
            for k in range(num[2]):
                x = left_bottom[0] + step[0] * i
                y = left_bottom[1] + step[1] * j
                z = left_bottom[2] + step[2] * k
                x = round(x, PRECISION)
                y = round(y, PRECISION)
                z = round(z, PRECISION)
                particles.append([x, y, z])
    
def save():
    with open(filename, "w") as f:
        for p in particles:
            f.write(f"{p[0]} {p[1]} {p[2]} {radius} {mass}\n")

def main():
    generate_cuboid([-0.5, 0, -0.5], [0.5, 1.0, 0.5], [20, 40, 20])
    generate_cuboid([0.5, 0, 0.5], [0.5, 1.0, 0.5], [20, 40, 20])
    save()

if __name__ == "__main__":
    main()

