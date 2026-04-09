with open("stream_server/index.html", "r") as f:
    content = f.read()

import os
old_html = """<!-- Detected Markers Box -->
        <div class="tool-box flex-grow">
            <h3>Detected Markers (Real-time)</h3>"""

new_html = """<!-- Detected Markers Box -->
        <div class="tool-box flex-grow">
            <h3>Detected Markers (Real-time)</h3>
            <p style="font-size: 13px; color: #555; margin-bottom: 10px;">
                <em>Note on visual coordinate axes:</em> OpenCV assumes the camera image is formatted in BGR, but it is mapped from RGB. This swaps Red and Blue colors when rendering vectors. Therefore, the <strong>X-axis is rendered in Blue</strong>, strictly matching typical Red axis logic, and the <strong>Z-axis (pointing perpendicular/OUT of the tag surface) evaluates as Red</strong> instead of typical Blue logic. The Y-axis remains Green.
            </p>"""

if old_html in content:
    content = content.replace(old_html, new_html)
    with open("stream_server/index.html", "w") as f:
        f.write(content)
    print("index.html documentation added")
else:
    print("could not add html note")
