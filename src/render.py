import bpy, mathutils
import bmesh
import numpy as np
import trimesh
import argparse

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
    def __init__(self, camera_location=(0, -0.25, 7), 
                camera_rotation=(0, 0, 0),
                bg_color=(0, 0, 0, 1), 
                light_location=(0, 5, -6), 
                light_energy=2000):
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete()

        bpy.ops.object.camera_add(location=camera_location, rotation=camera_rotation)
        camera = bpy.context.object
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

        bpy.ops.object.light_add(type='POINT', location=light_location)
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
    renderer = Render()
    renderer.render_coupled_fluid_rigid('fluid_mesh.obj', 'cloth_mesh.obj', None, 'output.png')

if __name__ == "__main__":
    main()