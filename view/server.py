from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
port = 8080

class myHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()
        self.wfile.write("Hello, World\n")
        self.wfile.write("your path is %s" % self.path)
        return

server = HTTPServer(("localhost", port), myHandler)
server.serve_forever()