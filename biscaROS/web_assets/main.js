const mainStream = document.getElementById('main-stream');

window.addEventListener('load', () => {
    mainStream.src = `${window.location.protocol}//${window.location.hostname}:7124/stream/main`;
    setInterval(fetchArucoData, 200);

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
        }).catch(err => console.error("Error updating vision layers:", err));
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
            } else {
                statusText.innerText = "Detection Disabled";
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
            }
        })
        .catch(err => console.error("Error fetching ArUco data:", err));
}
