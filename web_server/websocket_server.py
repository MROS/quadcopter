import sys
sys.path.append("..")
from quadcopter import quadcopter
import tornado.websocket
import tornado.ioloop
import tornado.httpserver
import tornado.web


class WSHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        print('new connection\n')
        self.write_message("Hello, World")
    def on_message(self, message):
        print('get message %s\n')


application = tornado.web.Application([(r'/ws', WSHandler), ])


if __name__ == "__main__":
    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen(8888)
    tornado.ioloop.IOLoop.instance().start()
