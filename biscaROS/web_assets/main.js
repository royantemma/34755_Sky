// web_assets/main.js
const mainStream = document.getElementById('main-stream');

window.addEventListener('load', () => {
    mainStream.src = `${window.location.protocol}//${window.location.hostname}:7124/stream`;
    setInterval(fetchRobotData, 200);

    // Vision Layer Toggles
    const layerAruco = document.getElementById('layer-aruco');
    const layerRed = document.getElementById('layer-red');
    const layerBW = document.getElementById('layer-bw');

    function updateLayers() {
        const layers = {
            aruco: layerAruco.checked,
            red_mask: layerRed.checked,
            bw_threshold: layerBW.checked
        };
        fetch('/api/vision/layers', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(layers)
        });
    }

    layerAruco.addEventListener('change', updateLayers);
    layerRed.addEventListener('change', updateLayers);
    layerBW.addEventListener('change', updateLayers);
});

const cameraBox = document.getElementById('camera-box');
const box3d = document.getElementById('box-3d');
new ResizeObserver(() => {
    if (cameraBox && box3d) {
        box3d.style.height = cameraBox.offsetHeight + 'px';
    }
}).observe(cameraBox);

// UI Elements
const toggleBtn = document.getElementById('aruco-toggle');
const statusText = document.getElementById('aruco-status-text');
const tbody = document.getElementById('aruco-table-body');
const tbodyEst = document.getElementById('aruco-estimates-body');
const poseInfo = document.getElementById('averaged-pose-info');
const poseCoords = document.getElementById('averaged-pose-coords');
const cubeInfo = document.getElementById('cube-center-info');
const cubeCoords = document.getElementById('cube-center-coords');

// ArUco Toggle
toggleBtn.addEventListener('change', (e) => {
    const enabled = e.target.checked;
    fetch('/api/cmd/aruco_toggle', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ enabled: enabled })
    });
});

// Mission Commands
function startMission(missionName) {
    fetch('/api/cmd/mission', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ mission: missionName })
    });
}

document.getElementById('btn-cube-catch').addEventListener('click', () => startMission('cube_catch_mission'));
document.getElementById('btn-complete-mission').addEventListener('click', () => startMission('complete_mission'));

document.getElementById('stop-btn').addEventListener('click', () => {
    fetch('/api/cmd/abort', { method: 'POST' });
});

document.getElementById('disable-servo-btn').addEventListener('click', () => {
    fetch('/api/cmd/servo', { method: 'POST' });
});

// Polling and DOM Updates
function fetchRobotData() {
    fetch('/api/data')
        .then(res => res.json())
        .then(state => {
            const enabled = state.aruco_enabled;
            
            if (toggleBtn.checked !== enabled) toggleBtn.checked = enabled;
            statusText.innerText = enabled ? "Detection Enabled" : "Detection Disabled";

            if (!enabled) {
                tbody.innerHTML = '<tr><td colspan="4">Detection disabled</td></tr>';
                tbodyEst.innerHTML = '<tr><td colspan="4">No estimates available</td></tr>';
                poseInfo.classList.add('d-none');
                cubeInfo.classList.add('d-none');
                return;
            }

            const arucoData = state.aruco_data || {};

            // Markers Table
            if (arucoData.markers && arucoData.markers.length > 0) {
                tbody.innerHTML = arucoData.markers.map(m => `
                    <tr>
                        <td><strong>${m.id}</strong> (${m.type})</td>
                        <td>${Math.round(m.x)}</td>
                        <td>${Math.round(m.y)}</td>
                        <td>${m.z !== undefined ? Math.round(m.z) : 'N/A'}</td>
                    </tr>
                    ${m.type === 'static' ? `
                    <tr><td colspan="4" style="font-size:12px; color: #666; background: #fafafa;">
                        Yaw: ${Math.round(m.yaw * 180 / Math.PI)}&deg; | 
                        Pitch: ${Math.round(m.pitch * 180 / Math.PI)}&deg; | 
                        Roll: ${Math.round(m.roll * 180 / Math.PI)}&deg;
                    </td></tr>` : ''}
                `).join('');
            } else {
                tbody.innerHTML = '<tr><td colspan="4">No markers detected</td></tr>';
            }
            
            // Cube Information
            if (arucoData.cube_center) {
                cubeInfo.classList.remove('d-none');
                cubeCoords.innerText = `X=${Math.round(arucoData.cube_center.x)} mm, Y=${Math.round(arucoData.cube_center.y)} mm`;
            } else {
                cubeInfo.classList.add('d-none');
            }
            
            // Position Estimates Table & Averaging
            if (arucoData.estimates && arucoData.estimates.length > 0) {
                tbodyEst.innerHTML = arucoData.estimates.map(e => `
                    <tr>
                        <td><strong>${e.marker_id}</strong></td>
                        <td>${Math.round(e.robot_x)}</td>
                        <td>${Math.round(e.robot_y)}</td>
                        <td>${Math.round(e.robot_yaw * 180 / Math.PI)}</td>
                    </tr>
                `).join('');
                
                let sumX = 0, sumY = 0, sumSin = 0, sumCos = 0;
                arucoData.estimates.forEach(e => {
                    sumX += e.robot_x;
                    sumY += e.robot_y;
                    sumSin += Math.sin(e.robot_yaw);
                    sumCos += Math.cos(e.robot_yaw);
                });
                
                const avgX = sumX / arucoData.estimates.length;
                const avgY = sumY / arucoData.estimates.length;
                const avgYaw = Math.atan2(sumSin, sumCos);
                
                poseInfo.classList.remove('d-none');
                poseCoords.innerText = `X: ${Math.round(avgX)} mm, Y: ${Math.round(avgY)} mm, Yaw: ${Math.round(avgYaw * 180 / Math.PI)} deg`;
                
                // Dispatch event to update the 3D model
                window.dispatchEvent(new CustomEvent('robotPoseUpdate', {
                    detail: { x: avgX, y: avgY, yaw: avgYaw }
                }));
            } else {
                tbodyEst.innerHTML = '<tr><td colspan="4">No estimates available</td></tr>';
                poseInfo.classList.add('d-none');
            }
        });
}