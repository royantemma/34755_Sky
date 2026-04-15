with open("stream_server/index.html", "r") as f:
    content = f.read()

old_block = "        <!-- 3D Controls Box -->"

new_block = """        <!-- Position Estimates Box -->
        <div class="tool-box flex-grow">
            <h3>Position Estimates (from ArUco)</h3>
            <div class="table-container">
                <table>
                    <thead>
                        <tr>
                            <th>From ID</th>
                            <th>X (mm)</th>
                            <th>Y (mm)</th>
                            <th>Yaw (deg)</th>
                        </tr>
                    </thead>
                    <tbody id="aruco-estimates-body">
                        <tr><td colspan="4">No estimates available</td></tr>
                    </tbody>
                </table>
            </div>
            <div id="averaged-pose-info" class="mt-15 info-box d-none">
                Averaged Pose:<br><span id="averaged-pose-coords">N/A</span>
            </div>
        </div>
        
        <!-- 3D Controls Box -->"""

if old_block in content:
    content = content.replace(old_block, new_block)
    with open("stream_server/index.html", "w") as f:
        f.write(content)
        print("index.html patched")
else:
    print("Could not find old_block in index.html")
