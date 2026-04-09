with open("stream_server/main.js", "r") as f:
    content = f.read()

# I need to update the main.js HTML replacement block to add the yaw text
# Looking for `<td><strong>${m.id}</strong> (${m.type})</td>`
old_block = """                        <td><strong>${m.id}</strong> (${m.type})</td>
                        <td>${Math.round(m.x)}</td>
                        <td>${Math.round(m.y)}</td>
                        <td>${Math.round(m.z)}</td>
                    </tr>"""
                    
new_block = """                        <td><strong>${m.id}</strong> (${m.type})</td>
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
                    ` : ''}"""

if old_block in content:
    content = content.replace(old_block, new_block)
    with open("stream_server/main.js", "w") as f:
        f.write(content)
    print("main.js patched with UI rotation data")
else:
    print("main.js replace failed")
