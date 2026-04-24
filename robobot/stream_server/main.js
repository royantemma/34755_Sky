const mainStream = document.getElementById('main-stream');
const cameratestStream = document.getElementById('cameratest-stream');

window.addEventListener('load', () => {
    mainStream.src = '/stream/main';
    setInterval(fetchArucoData, 200);
    setInterval(fetchIwoData, 200);
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

// Mission buttons
document.getElementById('start-mission-btn').addEventListener('click', () => {
    fetch('/api/mission/start').catch(() => alert("Error starting mission."));
});
document.getElementById('stop-mission-btn').addEventListener('click', () => {
    fetch('/api/mission/stop').catch(() => alert("Error stopping mission."));
});
document.getElementById('taskset-1-btn').addEventListener('click', () => {
    fetch('/api/mission/set?taskset=1').catch(() => alert("Error setting Task Set 1."));
});

document.getElementById('taskset-2-btn').addEventListener('click', () => {
    fetch('/api/mission/set?taskset=2').catch(() => alert("Error setting Task Set 2."));
});

document.getElementById('taskset-3-btn').addEventListener('click', () => {
    fetch('/api/mission/set?taskset=3').catch(() => alert("Error setting Task Set 3."));
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


function fetchIwoData() {
    console.log('fetchIwoData called');  // Debug: check if function runs
    fetch('/api/iwo/data')
        .then(res => {
            console.log('IWO fetch response status:', res.status);  // Debug: check response
            return res.json();
        })
        .then(data => {
            console.log('IWO data received:', data);  // Debug: log the actual data
            if (data) {
                const imuReading = data;  // Single object, not an array
                
                // Convert from meters to millimeters
                const avgX = (imuReading.robot_x || 0) * 1000;   // meters to mm
                const avgY = (imuReading.robot_y || 0) * 1000;   // meters to mm
                const avgZ = (imuReading.robot_z || 0) * 1000;   // meters to mm
                const avgYaw = imuReading.robot_yaw || 0;        // already in radians
                const avgPitch = imuReading.robot_pitch || 0;    // already in radians
                const avgRoll = imuReading.robot_roll || 0;      // already in radians
                
                console.log('Dispatching robotPoseUpdate:', { x: avgX, y: avgY, z: avgZ, yaw: avgYaw, pitch: avgPitch, roll: avgRoll });  // Debug
                
                // Dispatch event for app3d.js with IWO pose
                window.dispatchEvent(new CustomEvent('robotPoseUpdate', {
                    detail: { x: avgX, y: avgY, z: avgZ, yaw: avgYaw, pitch: avgPitch, roll: avgRoll }
                }));
            }
        })
        .catch(err => console.error("Error fetching IWO data:", err));
}

// function fetchIwoData() {
//     fetch('/api/iwo/data')
//         .then(res => res.json())
//         .then(data => {
//             // data structure is now { 'data': { robot_x, robot_y, robot_z, robot_roll, robot_pitch, robot_yaw } }
//             if (data.data) {
//                 const imuReading = data.data;  // Single object, not an array
                
//                 // Convert from meters to millimeters
//                 const avgX = (imuReading.robot_x || 0) * 1000;   // meters to mm
//                 const avgY = (imuReading.robot_y || 0) * 1000;   // meters to mm
//                 const avgZ = (imuReading.robot_z || 0) * 1000;   // meters to mm
//                 const avgYaw = imuReading.robot_yaw || 0;        // already in radians
//                 const avgPitch = imuReading.robot_pitch || 0;    // already in radians
//                 const avgRoll = imuReading.robot_roll || 0;      // already in radians
                
//                 // Dispatch event for app3d.js with IWO pose
//                 window.dispatchEvent(new CustomEvent('robotPoseUpdate', {
//                     detail: { x: avgX, y: avgY, z: avgZ, yaw: avgYaw, pitch: avgPitch, roll: avgRoll }
//                 }));
//             }
//         })
//         .catch(err => console.error("Error fetching IWO data:", err));
// }

// // New function to fetch IMU data (assuming Python serves it via /api/imu/data with similar structure to ArUco estimates)
// function fetchIwoData() {
//     fetch('/api/iwo/data')  // Change endpoint to IMU data source
//         .then(res => res.json())
//         .then(data => {
//             // Assume data has 'estimates' array with IMU pose data
//             // For simplicity, treat as single or averaged IMU reading
//             if (data.estimates && data.estimates.length > 0) {
//                 // Compute average if multiple, similar to ArUco
//                 // let sumX = 0, sumY = 0, sumZ = 0, sumSinYaw = 0, sumCosYaw = 0, sumSinPitch = 0, sumCosPitch = 0, sumSinRoll = 0, sumCosRoll = 0;
//                 // data.estimates.forEach(e => {
//                 //     sumX += e.robot_x;
//                 //     sumY += e.robot_y;
//                 //     sumZ += e.robot_z;
//                 //     sumSinYaw += Math.sin(e.robot_yaw);
//                 //     sumCosYaw += Math.cos(e.robot_yaw);
//                 //     sumSinPitch += Math.sin(e.robot_pitch);
//                 //     sumCosPitch += Math.cos(e.robot_pitch);
//                 //     sumSinRoll += Math.sin(e.robot_roll);
//                 //     sumCosRoll += Math.cos(e.robot_roll);
//                 // });
                
//                 // const dec = data.estimates.length;

//                 const avgX = roboto_x;
//                 const avgY = robot_y;
//                 const avgZ = robot_z;
//                 const avgYaw = robot_yaw;
//                 const avgPitch = robot_pitch;
//                 const avgRoll = robot_roll;
//                 // const avgYaw = Math.atan2(sumSinYaw, sumCosYaw);
//                 // const avgPitch = Math.atan2(sumSinPitch, sumCosPitch);
//                 // const avgRoll = Math.atan2(sumSinRoll, sumCosRoll);
                
//                 // Dispatch event for app3d.js with IMU pose
//                 window.dispatchEvent(new CustomEvent('robotPoseUpdate', {
//                     detail: { x: avgX, y: avgY, z: avgZ, yaw: avgYaw, pitch: avgPitch, roll: avgRoll }
//                 }));
//             }
//             // Removed UI updates for markers, cube, etc., as they are ArUco-specific
//         })
//         .catch(err => console.error("Error fetching IMU data:", err));
// }