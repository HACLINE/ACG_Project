import bpy, mathutils
import bmesh
import trimesh
import yaml
import os

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

def trimesh_to_blender_object(trimesh_obj, object_name="Bunny"):
    mesh = bpy.data.meshes.new(object_name)
    bm = bmesh.new()

    for vertex in trimesh_obj.vertices:
        bm.verts.new(vertex)
    bm.verts.ensure_lookup_table()

    existing_faces = set()
    for face in trimesh_obj.faces:
        face_tuple = tuple(sorted(face))
        if face_tuple in existing_faces:
            continue
        try:
            bm.faces.new([bm.verts[i] for i in face])
            existing_faces.add(face_tuple)
        except ValueError:
            continue
    bm.faces.ensure_lookup_table()
    bm.to_mesh(mesh)
    bm.free()

    obj = bpy.data.objects.new(object_name, mesh)
    bpy.context.collection.objects.link(obj)

    if not obj.data.materials:
        mat = bpy.data.materials.new(name="DoubleSidedMaterial")
        obj.data.materials.append(mat)
    else:
        mat = obj.data.materials[0]
    
    mat.use_backface_culling = False  

    # mesh_info = {
    #     "name": obj.name,
    #     "vertices": len(obj.data.vertices),
    #     "edges": len(obj.data.edges),
    #     "faces": len(obj.data.polygons)
    # }
    # print(f"Mesh Info: {mesh_info}")

    return obj

class Render:
    def __init__(self, config):#camera_location=(0, -0.25, 7), 
                # camera_rotation=(0, 0, 0),
                # bg_color=(0, 0, 0, 1), 
                # light_location=(0, 5, -6), 
                # light_energy=2000):
#         render:
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
        light_location = tuple(config["render"]["light"]["position"][:3])
        light_energy = config["render"]["light"]["energy"]

        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete()

        bpy.ops.object.camera_add(location=camera_location, rotation=camera_rotation)
        camera = bpy.context.object
        camera.data.lens = config["render"]["perspective"]["fovy"]
        camera.data.sensor_width = config["render"]["perspective"]["sensor_width"]
        camera.data.sensor_height = config["render"]["perspective"]["sensor_width"] / config["render"]["perspective"]["aspect"]
        camera.data.clip_start = config["render"]["perspective"]["znear"]
        camera.data.clip_end = config["render"]["perspective"]["zfar"]
        camera.data.lens_unit = 'FOV'

        bpy.context.scene.camera = camera
        bpy.context.scene.world.use_nodes = True
        world = bpy.context.scene.world
        node_tree = world.node_tree
        nodes = node_tree.nodes
        links = node_tree.links
        for node in nodes:
            nodes.remove(node)

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
            bpy.ops.object.light_add(type='POINT', location=light_location)
        else:
            bpy.ops.object.light_add(type='SUN', location=light_location)
        light = bpy.context.object
        light.data.energy = light_energy
        
        self.fluid_mesh = []
        
    def look_at(self, obj_camera, point):
        loc_camera = obj_camera.matrix_world.to_translation()

        direction = point - loc_camera
        rot_quat = direction.to_track_quat('-Z', 'Y')

        obj_camera.rotation_euler = rot_quat.to_euler()
        
    def render_mesh(self, mesh_list:list, output_path):    
        for mesh in mesh_list:
            bpy.context.view_layer.objects.active = mesh
            mesh.select_set(True)
        
        bpy.context.scene.render.resolution_x = self.config["render"]["resolution"]["x"]
        bpy.context.scene.render.resolution_y = self.config["render"]["resolution"]["y"]
        bpy.context.scene.render.resolution_percentage = self.config["render"]["resolution"]["percentage"]

        bpy.context.scene.render.image_settings.file_format = 'PNG'
        bpy.context.scene.render.filepath = output_path
        bpy.ops.render.render(write_still=True)
        
        return output_path
        
    def render_coupled_fluid_rigid(self, fluid_mesh_path, rigid_mesh_path, container_mesh_path, output_path):
        bpy.ops.object.select_all(action='DESELECT')
        for obj in bpy.data.objects:
            if obj.type == 'MESH':
                obj.select_set(True)
        bpy.ops.object.delete()

        fluid_mesh = trimesh.load(fluid_mesh_path)

        fluid_material = self.get_material("Water", "assets/water.blend")
        
        fluid_mesh = trimesh_to_blender_object(fluid_mesh, object_name="Fluid")
        if fluid_mesh.data.materials:
            fluid_mesh.data.materials[0] = fluid_material
        else:
            fluid_mesh.data.materials.append(fluid_material)

        rigid_mesh = trimesh.load(rigid_mesh_path)

        rigid_material = self.get_material('Realistic procedural gold', 'assets/rigid.blend')
        rigid_mesh = trimesh_to_blender_object(rigid_mesh, object_name="Rigid")
        if rigid_mesh.data.materials:
            rigid_mesh.data.materials[0] = rigid_material
        else:
            rigid_mesh.data.materials.append(rigid_material)
        
        # container_mesh = trimesh.load(container_mesh_path)
        # container_mesh = self.add_container(container_mesh)
        self.render_mesh([fluid_mesh, rigid_mesh], output_path)

    def render_all(self, data_path, container_mesh_path, output_path):
        bpy.ops.object.select_all(action='DESELECT')
        for obj in bpy.data.objects:
            if obj.type == 'MESH':
                obj.select_set(True)
        bpy.ops.object.delete()

        all_mesh = []

        fluid_material = self.get_material("Water", "assets/water.blend")
        if self.config["load"]["fluid"]["cfg"] is not None:
            for i in range(len(self.config["load"]["fluid"]["cfg"])):
                all_mesh.append(trimesh.load(data_path + "/fluids/" + int_to_string(i, 2) + ".obj"))
                all_mesh[-1] = trimesh_to_blender_object(all_mesh[-1], object_name="Fluid" + int_to_string(i, 2))
                if all_mesh[-1].data.materials:
                    all_mesh[-1].data.materials[0] = fluid_material
                else:
                    all_mesh[-1].data.materials.append(fluid_material)
        
        cloth_material = self.get_material('Realistic procedural gold', 'assets/rigid.blend')
        if self.config["load"]["cloth"]["cfg"] is not None:
            for i in range(len(self.config["load"]["cloth"]["cfg"])):
                all_mesh.append(trimesh.load(data_path + "/cloths/" + int_to_string(i, 2) + ".obj"))
                all_mesh[-1] = trimesh_to_blender_object(all_mesh[-1], object_name="Cloth" + int_to_string(i, 2))
                if all_mesh[-1].data.materials:
                    all_mesh[-1].data.materials[0] = cloth_material
                else:
                    all_mesh[-1].data.materials.append(cloth_material)
        
        self.render_mesh(all_mesh, output_path)
            
        
    def add_container(self, container_mesh):
        glass_material = self.get_material("Scratched Glass (Procedural)", "assets/glass.blend")
        
        container_mesh = trimesh_to_blender_object(container_mesh, object_name="Container")
        if container_mesh.data.materials:
            container_mesh.data.materials[0] = glass_material
        else:
            container_mesh.data.materials.append(glass_material)
        return container_mesh
        
    def get_material(self, material_name, file_name):
        blend_file_path = file_name
        with bpy.data.libraries.load(blend_file_path, link=False) as (data_from, data_to):
            if material_name in data_from.materials:
                data_to.materials = [material_name]
            else:
                raise ValueError(f"Material {material_name} not found in {blend_file_path}")

        return data_to.materials[0]
    
def main():
    assert 'CONFIG' in os.environ, "CONFIG environment variable not set"
    assert 'RENDER' in os.environ, "RENDER environment variable not set"
    config = load_config(os.environ['CONFIG'])
    renderer = Render(config)

    for i in range(0, config["video"]["sps"] * config["video"]["length"], config["video"]["sps"] // config["video"]["fps"]):
        renderer.render_all("./.cache/frame_" + int_to_string(i, 6), None, os.environ['RENDER'] + "/frame_" + int_to_string(i, 6) + ".png")

if __name__ == "__main__":
    main()