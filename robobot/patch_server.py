with open("stream_server/stream_server.py", "r") as f:
    content = f.read()

old_block = """        else:
            self.send_error(404)
            self.end_headers()"""

new_block = """        else:
            try:
                import mimetypes
                req_path = self.path.split('?')[0].lstrip('/')
                if '..' in req_path:
                    self.send_error(403)
                    return
                curr_dir = os.path.dirname(os.path.abspath(__file__))
                filepath = os.path.join(curr_dir, req_path)
                if os.path.isfile(filepath):
                    mimetype, _ = mimetypes.guess_type(filepath)
                    with open(filepath, 'rb') as f:
                        file_content = f.read()
                    self.send_response(200)
                    self.send_header('Content-Type', mimetype or 'application/octet-stream')
                    self.send_header('Content-Length', len(file_content))
                    self.end_headers()
                    self.wfile.write(file_content)
                else:
                    self.send_error(404)
            except Exception as e:
                self.send_error(500, f"Error: {e}")"""

if old_block in content:
    content = content.replace(old_block, new_block)
    with open("stream_server/stream_server.py", "w") as f:
        f.write(content)
    print("Patched successfully")
else:
    print("Could not find old block")
