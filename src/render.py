import bpy, mathutils
import bmesh
import trimesh
import yaml
import os

DEFAULT_MESH_MATERIAL = "DoubleSidedMaterial"
SPHERE_MATERIAL = "SphereMaterial"
FLUID_MATERIAL = "Water"
CLOTH_MATERIAL = "Realistic procedural gold"
CONTAINER_MATERIAL = "Scratched Glass (Procedural)"

FLUID_MAT_PATH = "assets/water.blend"
CLOTH_MAT_PATH = "assets/rigid.blend"
CONTAINER_MAT_PATH = "assets/glass.blend"

def load_config(config_path):
    def merge(base, update):
        for key, value in update.items():
            if key in base and isinstance(base[key], dict) and isinstance(value, dict):
                base[key] = merge(base[key], value)
            else:
                base[key] = value
        return base

    with open("./config/" + config_path, 'r') as file:
        config = yaml.load(file, Loader=yaml.FullLoader)
    with open("./config/" + config["base"], 'r') as file:
        base_config = yaml.load(file, Loader=yaml.FullLoader)
    config = merge(base_config, config)
    with open("./config/" + config["load"]["fluid"]["base"], 'r') as file:
        base_fluid_config = yaml.load(file, Loader=yaml.FullLoader)
    config["load"]["fluid"] = merge(base_fluid_config, config["load"]["fluid"])
    with open("./config/" + config["load"]["cloth"]["base"], 'r') as file:
        base_cloth_config = yaml.load(file, Loader=yaml.FullLoader)
    config["load"]["cloth"] = merge(base_cloth_config, config["load"]["cloth"])
    return config

def int_to_string(num, length):
    num = str(num)
    while len(num) < length:
        num = "0" + num
    return num

def mesh_to_blender(obj, name):
    obj_mesh = bpy.data.meshes.new(name)
    
    # process mesh using bmesh
    _mesh = bmesh.new()
    for v in obj.vertices: 
        _mesh.verts.new(v)
    _mesh.verts.ensure_lookup_table()
    faces = set()
    for f in obj.faces:
        t = tuple(sorted(f))
        if f not in faces: # eliminate duplicate faces
            try:
                _mesh.faces.new([_mesh.verts[i] for i in f])
                faces.add(t)
            except ValueError:
                print("[Blender] Encountered value error when processing face.")
    _mesh.faces.ensure_lookup_table()
    _mesh.to_mesh(obj_mesh)
    _mesh.free()

    blender_object = bpy.data.objects.new(name, obj_mesh)
    
    bpy.context.collection.objects.link(blender_object)
    if not obj.data.materials:
        material = bpy.data.materials.new(name=DEFAULT_MESH_MATERIAL)
        blender_object.data.materials.append(material)
    else:
        material = blender_object.data.materials[0]
    material.use_backface_culling = False
    
    assert name == blender_object.name
    print(f"[Blender] Mesh name: {name} has been added to the scene. Info: {len(blender_object.data.vertices)} vertices, {len(blender_object.data.edges)} edges, {len(blender_object.data.polygons)} faces.")

    return obj

def render_sphere_from_file(file_path, radius_offset=0.02):
    with open(file_path, 'r') as file:
        line = file.readline().strip()
        x, y, z, radius = map(float, line.split())
        bpy.ops.mesh.primitive_uv_sphere_add(radius=radius - radius_offset, location=(x, y, z), segments=64, ring_count=32)
        sphere = bpy.context.object
        material = bpy.data.materials.new(name=SPHERE_MATERIAL)
        material.diffuse_color = (1, 0, 0, 1)  # Red color
        sphere.data.materials.append(material)
    return sphere

def process_material(mesh, material):
        if mesh.data.materials:
            mesh.data.materials[0] = material
        else:
            mesh.data.materials.append(material)

class Render:
    def __init__(self, config):
# render:
#   init:
#     argc: 0
#     argv: []
#   camera:
#     eye: [15.0, 9.0, 15.0]
#     center: [0.0, 0.0, 0.0]
#     up: [0.0, 1.0, 0.0]
#   title: Basic
#   windowsize: [800, 600]
#   viewport: [0, 0, 800, 600]
#   perspective: 
#     fovy: 45.0
#     aspect: 1.3333
#     znear: 1.0
#     zfar: 1000.0
#   clearcolor: [1.0, 1.0, 1.0, 1.0]
#   light:
#     position: [0.5, 1.0, 0.0, 0.0]
#     ambient: [0.2, 0.2, 0.2, 1.0]
#     diffuse: [0.8, 0.8, 0.8, 1.0]
#     specular: [1.0, 1.0, 1.0, 1.0]
        self.config = config

        camera_location = tuple(config["render"]["camera"]["eye"])
        direction = mathutils.Vector(tuple(config["render"]["camera"]["center"])) - mathutils.Vector(camera_location)
        world_up = mathutils.Vector(tuple(config["render"]["camera"]["up"]))
        right = direction.cross(world_up)
        right.normalize()
        up = right.cross(direction)
        up.normalize()
        rotation_matrix = mathutils.Matrix((
            right,
            up,
            -direction
        )).transposed()
        camera_rotation = rotation_matrix.to_euler()

        bg_color = tuple(config["render"]["background"]["color"])
        light_position = tuple(config["render"]["light"]["position"][:3])
        light_energy = config["render"]["light"]["energy"]

        bpy.ops.object.select_all(action='DESELECT')
        for obj in bpy.context.scene.objects:
            obj.select_set(True)
        bpy.ops.object.delete()
        
        camera = bpy.context.object
        camera.data.lens = config["render"]["perspective"]["render_fovy"]
        camera.data.sensor_width = config["render"]["perspective"]["sensor_width"]
        camera.data.sensor_height = config["render"]["perspective"]["sensor_width"] / config["render"]["perspective"]["aspect"]
        camera.data.clip_start = config["render"]["perspective"]["znear"]
        camera.data.clip_end = config["render"]["perspective"]["zfar"]
        camera.data.lens_unit = 'FOV'

        bpy.context.scene.camera, bpy.context.scene.world.use_nodes = camera, True
        node_tree = bpy.context.scene.world.node_tree
        nodes, links = node_tree.nodes, node_tree.links
        for _ in nodes:
            nodes.remove(_)

        bg_node = nodes.new(type='ShaderNodeBackground')
        bg_node.inputs['Color'].default_value = bg_color
        env_texture_node = nodes.new(type='ShaderNodeTexEnvironment')
        env_texture_node.image = bpy.data.images.load('assets/background.hdr')

        output_node = nodes.new(type='ShaderNodeOutputWorld')

        links.new(env_texture_node.outputs['Color'], bg_node.inputs['Color'])
        links.new(bg_node.outputs['Background'], output_node.inputs['Surface'])
        bg_node = bpy.context.scene.world.node_tree.nodes['Background']
        bg_node.inputs['Color'].default_value = bg_color

        if config["render"]["light"]["position"][3] == 1:
            bpy.ops.object.light_add(type='POINT', location=light_position)
        else:
            bpy.ops.object.light_add(type='SUN', location=light_position)
            
        bpy.context.object.data.energy = light_energy
        
    def look_at(self, obj_camera, point):
        loc_camera = obj_camera.matrix_world.to_translation()

        direction = point - loc_camera
        rot_quat = direction.to_track_quat('-Z', 'Y')

        obj_camera.rotation_euler = rot_quat.to_euler()
        
    def render_mesh(self, meshes: list, pth):    
        for mesh in meshes:
            bpy.context.view_layer.objects.active = mesh
            mesh.select_set(True)
        
        bpy.context.scene.render.resolution_x = self.config["render"]["resolution"]["x"]
        bpy.context.scene.render.resolution_y = self.config["render"]["resolution"]["y"]
        bpy.context.scene.render.resolution_percentage = self.config["render"]["resolution"]["percentage"]

        bpy.context.scene.render.image_settings.file_format = 'PNG'
        bpy.context.scene.render.filepath = pth
        bpy.ops.render.render(write_still=True)

    def render_all(self, data_path, container_mesh_path, output_path):
        bpy.ops.object.select_all(action='DESELECT')
        for obj in bpy.data.objects:
            if obj.type == 'MESH':
                obj.select_set(True)
        bpy.ops.object.delete()

        all_mesh = []

        fluid_material = self.get_material(FLUID_MATERIAL, FLUID_MAT_PATH)
        if self.config["load"]["fluid"]["cfg"] is not None:
            for i in range(len(self.config["load"]["fluid"]["cfg"])):
                all_mesh.append(trimesh.load(data_path + "/fluids/" + int_to_string(i, 2) + ".obj"))
                all_mesh[-1] = mesh_to_blender(all_mesh[-1], "Fluid" + int_to_string(i, 2))
                process_material(all_mesh[-1], fluid_material)
        
        cloth_material = self.get_material(CLOTH_MATERIAL, CLOTH_MAT_PATH)
        if self.config["load"]["cloth"]["cfg"] is not None:
            for i in range(len(self.config["load"]["cloth"]["cfg"])):
                all_mesh.append(trimesh.load(data_path + "/cloths/" + int_to_string(i, 2) + ".obj"))
                all_mesh[-1] = mesh_to_blender(all_mesh[-1], "Cloth" + int_to_string(i, 2))
                process_material(all_mesh[-1], cloth_material)
        
        sphere_files = [f for f in os.listdir(data_path + "/spheres") if f.endswith(".sphere")]
        for sphere_file in sphere_files:
            render_sphere_from_file(data_path + "/spheres/" + sphere_file)
        
        self.render_mesh(all_mesh, output_path)
            
        
    def add_container(self, container_mesh):
        glass_material = self.get_material(CONTAINER_MATERIAL, CONTAINER_MAT_PATH)
        container_mesh = mesh_to_blender(container_mesh, "Container")
        process_material(container_mesh, glass_material)
        return container_mesh
        
    def get_material(self, mat_name, file_name):
        with bpy.data.libraries.load(file_name, link=False) as (df, dt):
            if mat_name not in df.materials:
                raise ValueError(f"Material {mat_name} not found in {file_name}")
            dt.materials = [mat_name]
        return dt.materials[0]
    
def main():
    assert 'CONFIG' in os.environ, "CONFIG environment variable not set"
    assert 'RENDER' in os.environ, "RENDER environment variable not set"
    config = load_config(os.environ['CONFIG'])
    renderer = Render(config)

    for i in range(0, config["video"]["sps"] * config["video"]["length"], config["video"]["sps"] // config["video"]["fps"]):
        renderer.render_all("./.cache/frame_" + int_to_string(i, 6), None, os.environ['RENDER'] + "/frame_" + int_to_string(i, 6) + ".png")

if __name__ == "__main__":
    main()