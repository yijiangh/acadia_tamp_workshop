import bpy,os

def main():
    HERE = os.path.dirname(__file__)
    for root, dirs, files in os.walk(os.path.join(HERE, 'robotiq_85')):
        for file in files:
            if file.endswith(".dae"):
                fullpath = os.path.join(root, file)
                print(fullpath)
                bpy.ops.wm.collada_import(filepath=fullpath)
                bpy.ops.export_scene.obj(filepath=fullpath[:-4] + ".obj", use_materials=True, axis_forward='Y', axis_up='Z')
                bpy.ops.object.select_all(action='SELECT')
                bpy.ops.object.delete(use_global=False)

if __name__ == '__main__':
    main()