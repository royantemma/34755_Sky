with open("stream_server/app3d.js", "r") as f:
    content = f.read()

old_block = """loader.load('3D/robot.fbx', function (object) {
    robotModel.remove(placeholderMesh);
    robotModel.add(object);
}, undefined, function (error) {"""

new_block = """loader.load('3D/robot.fbx', function (object) {
    robotModel.remove(placeholderMesh);
    // Correct the intrinsic 180 degree yaw offset of the base robot model
    object.rotation.z = Math.PI; 
    robotModel.add(object);
}, undefined, function (error) {"""

if old_block in content:
    content = content.replace(old_block, new_block)
    with open("stream_server/app3d.js", "w") as f:
        f.write(content)
    print("app3d.js 3d model yaw patched")
else:
    print("Could not find block in app3d.js")
