const mainStream = document.getElementById('main-stream');
const cameratestStream = document.getElementById('cameratest-stream');

window.addEventListener('load', () => {
    mainStream.src = '/stream/main';
    setInterval(fetchArucoData, 200);
});

const cameraBox = document.getElementById('camera-box');
const box3d = document.getElementById('box-3d');
new ResizeObserver(() => {
    if (cameraBox && box3d) {
        box3d.style.height = cameraBox.offsetHeight + 'px';
    }
}).observe(cameraBox);

const toggleBtn = document.getElementById('aruco-toggle');
const statusText = document.getElementById('aruco-status-text');
const tbody = document.getElementById('aruco-table-body');

toggleBtn.addEventListener('change', (e) => {
    const enabled = e.target.checked ? 1 : 0;
    fetch(`/api/aruco/set?enabled=${enabled}`)
        .catch(err => console.error("Error setting ArUco state:", err));
});

document.getElementById('catch-btn').addEventListener('click', () => {
    fetch('/api/aruco/catch').catch(() => alert("Error starting mission."));
    if(!toggleBtn.checked) {
        toggleBtn.checked = true;
        fetch('/api/aruco/set?enabled=1');
    }
});

document.getElementById('stop-btn').addEventListener('click', () => {
    fetch('/api/aruco/stop').catch(() => alert("Error stopping mission."));
});

document.getElementById('disable-servo-btn').addEventListener('click', () => {
    fetch('/api/servo/disable').catch(() => alert("Error disabling servo."));
});

function fetchArucoData() {
    fetch('/api/aruco/data')
        .then(res => res.json())
        .then(data => {
            if (toggleBtn.checked !== data.enabled) {
                toggleBtn.checked = data.enabled;
            }

            if (data.enabled) {
                statusText.innerText = "Detection Enabled";
                if (!cameratestStream.src || !cameratestStream.src.includes('/stream/cameratest')) {
                    cameratestStream.src = '/stream/cameratest';
                }
                cameratestStream.classList.remove('d-none');
                mainStream.classList.add('d-none');
            } else {
                statusText.innerText = "Detection Disabled";
                cameratestStream.classList.add('d-none');
                mainStream.classList.remove('d-none');
            }

            if (!data.enabled) {
                tbody.innerHTML = '<tr><td colspan="4">Detection disabled</td></tr>';
                return;
            }

            if (data.markers && data.markers.length > 0) {
                tbody.innerHTML = data.markers.map(m => `
                    <tr>
                        <td><strong>${m.id}</strong> (${m.type})</td>
                        <td>${Math.round(m.x)}</td>
                        <td>${Math.round(m.y)}</td>
                        <td>${m.z !== undefined ? Math.round(m.z) : 'N/A'}</td>
                    </tr>
                    ${m.type === 'static' ? `
                    <tr><td colspan="4" style="font-size:12px; color: #666; background: #fafafa;">Orientation vs Robot Frame:
                        Yaw: ${Math.round(m.yaw * 180 / Math.PI)}&deg; | 
                        Pitch: ${Math.round(m.pitch * 180 / Math.PI)}&deg; | 
                        Roll: ${Math.round(m.roll * 180 / Math.PI)}&deg;
                    </td></tr>
                    ` : ''}
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
                        <td>${Math.round(e.robot_x)}, ${Math.round(e.robot_y)}, ${Math.round(e.robot_z)}</td>
                        <td>${Math.round(e.robot_yaw * 180 / Math.PI)}, ${Math.round(e.robot_pitch * 180 / Math.PI)}, ${Math.round(e.robot_roll * 180 / Math.PI)}</td>
                    </tr>
                `).join('');
                
                let sumX = 0, sumY = 0, sumZ = 0, sumSinYaw = 0, sumCosYaw = 0, sumSinPitch = 0, sumCosPitch = 0, sumSinRoll = 0, sumCosRoll = 0;
                data.estimates.forEach(e => {
                    sumX += e.robot_x;
                    sumY += e.robot_y;
                    sumZ += e.robot_z;
                    sumSinYaw += Math.sin(e.robot_yaw);
                    sumCosYaw += Math.cos(e.robot_yaw);
                    sumSinPitch += Math.sin(e.robot_pitch);
                    sumCosPitch += Math.cos(e.robot_pitch);
                    sumSinRoll += Math.sin(e.robot_roll);
                    sumCosRoll += Math.cos(e.robot_roll);
                });
                
                const dec = data.estimates.length;
                const avgX = sumX / dec;
                const avgY = sumY / dec;
                const avgZ = sumZ / dec;
                const avgYaw = Math.atan2(sumSinYaw, sumCosYaw);
                const avgPitch = Math.atan2(sumSinPitch, sumCosPitch);
                const avgRoll = Math.atan2(sumSinRoll, sumCosRoll);
                
                poseInfo.classList.remove('d-none');
                poseCoords.innerText = `X: ${Math.round(avgX)}, Y: ${Math.round(avgY)}, Z: ${Math.round(avgZ)}\nYaw: ${Math.round(avgYaw * 180 / Math.PI)}\u00b0, Pitch: ${Math.round(avgPitch * 180 / Math.PI)}\u00b0, Roll: ${Math.round(avgRoll * 180 / Math.PI)}\u00b0`;
                
                // Dispatch event for app3d.js
                window.dispatchEvent(new CustomEvent('robotPoseUpdate', {
                    detail: { x: avgX, y: avgY, z: avgZ, yaw: avgYaw, pitch: avgPitch, roll: avgRoll }
                }));
            } else {
                tbodyEst.innerHTML = '<tr><td colspan="3">No estimates available</td></tr>';
                poseInfo.classList.add('d-none');
            }
        })
        .catch(err => console.error("Error fetching ArUco data:", err));
}
