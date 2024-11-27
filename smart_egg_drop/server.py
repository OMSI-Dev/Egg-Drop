import os
from http.server import SimpleHTTPRequestHandler, HTTPServer

class CustomHandler(SimpleHTTPRequestHandler):
	def do_GET(self):
		if self.path == '/shutdown':
			self.send_response(200)
			self.end_headers()
			self.wfile.write(b"Shutdown initiated.")
			#shutdown pi
			os.system("sudo shutdown -h now")
		else:
			super().do_GET()
			
if __name__ == "__main__":
	server_address = ('', 8000)
	httpd = HTTPServer(server_address, CustomHandler)
	print("Server on port 8000")
	httpd.serve_forever()
