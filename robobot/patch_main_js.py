with open("stream_server/main.js", "r") as f:
    content = f.read()

# Find fetchArucoData inner block to replace
old_block = """            if (data.markers && data.markers.length > 0) {
                tbody.innerHTML = data.markers.map(m => `
                    <tr>
                        <td><strong>${m.id}</strong></td>
                        <td>${Math.round(m.x)}</td>
                        <td>${Math.round(m.y)}</td>
                        <td>${Math.round(m.z)}</td>
                    </tr>
                `).join('');
            } else {
                tbody.innerHTML = '<tr><td colspan="4">No markers detected</td></tr>';
            }
            
            const cubeInfo = document.getElementById('cube-center-info');
            const cubeCoords = document.getElementById('cube-center-coords');
            if (data.cube_center) {
                cubeInfo.classList.remove('d-none');
                cubeCoords.innerText = `X=${Math.round(data.cube_center.x)} mm, Y=${Math.round(data.cube_center.y)} mm, Z=${Math.round(data.cube_center.z)} mm`;
            } else {
                cubeInfo.classList.add('d-none');
            }"""

new_block = """            if (data.markers && data.markers.length > 0) {
                tbody.innerHTML = data.markers.map(m => `
                    <tr>
                        <td><strong>${m.id}</strong> (${m.type})</td>
                        <td>${Math.round(m.x)}</td>
                        <td>${Math.round(m.y)}</td>
                        <td>${Math.round(m.z)}</td>
                    </tr>
                `).join('');
            } else {
                tbody.innerHTML = '<tr><td colspan="4">No markers detected</td></tr>';
            }
            
            const cubeInfo = document.getElementById('cube-center-info');
            const cubeCoords = document.getElementById('cube-center-coords');
            if (data.cube_center) {
                cubeInfo.classList.remove('d-none');
                cubeCoords.innerText = `X=${Math.round(data.cube_center.x)} mm, Y=${Math.round(data.cube_center.y)} mm, Z=${Math.round(data.cube_center.z)} mm`;
            } else {
                cubeInfo.classList.add('d-none');
            }
            
            const tbodyEst = document.getElementById('aruco-estimates-body');
            const poseInfo = document.getElementById('averaged-pose-info');
            const poseCoords = document.getElementById('averaged-pose-coords');
            
            if (data.estimates && data.estimates.length > 0) {
                tbodyEst.innerHTML = data.estimates.map(e => `
                    <tr>
                        <td><strong>${e.marker_id}</strong></td>
                        <td>${Math.round(e.robot_x)}</td>
                        <td>${Math.round(e.robot_y)}</td>
                        <td>${Math.round(e.robot_yaw * 180 / Math.PI)}</td>
                    </tr>
                `).join('');
                
                let sumX = 0, sumY = 0, sumSin = 0, sumCos = 0;
                data.estimates.forEach(e => {
                    sumX += e.robot_x;
                    sumY += e.robot_y;
                    sumSin += Math.sin(e.robot_yaw);
                    sumCos += Math.cos(e.robot_yaw);
                });
                
                const avgX = sumX / data.estimates.length;
                const avgY = sumY / data.estimates.length;
                const avgYaw = Math.atan2(sumSin, sumCos);
                
                poseInfo.classList.remove('d-none');
                poseCoords.innerText = `X: ${Math.round(avgX)} mm, Y: ${Math.round(avgY)} mm, Yaw: ${Math.round(avgYaw * 180 / Math.PI)} deg`;
                
                // Dispatch event for app3d.js
                window.dispatchEvent(new CustomEvent('robotPoseUpdate', {
                    detail: { x: avgX, y: avgY, yaw: avgYaw }
                }));
            } else {
                tbodyEst.innerHTML = '<tr><td colspan="4">No estimates available</td></tr>';
                poseInfo.classList.add('d-none');
            }"""

if old_block in content:
    content = content.replace(old_block, new_block)
    with open("stream_server/main.js", "w") as f:
        f.write(content)
        print("main.js patched")
else:
    print("Could not patch main.js - block not found")
