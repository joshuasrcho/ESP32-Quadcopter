import tornado.ioloop
import tornado.web
import tornado.websocket
import base64
import sys
import os
import asyncio

settings = {
    "static_path": os.path.join(os.path.dirname(__file__), "static")
}

class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("joystick.html")

class WSHandler(tornado.websocket.WebSocketHandler):
    connections = set()
    def check_origin(self, origin):
        return True

    def open(self):
        self.connections.add(self)

    def on_message(self, message):
        [client.write_message(message) for client in FlightWSHandler.connections]
        print(message)

    def on_close(self):
        self.connections.remove(self)

class CameraWSHandler(tornado.websocket.WebSocketHandler):
    connections = set()
    def check_origin(self, origin):
        return True

    def open(self):
        self.connections.add(self)

    def on_message(self, message):
        base64Message = base64.b64encode(message)
        [client.write_message(base64Message) for client in WSHandler.connections]

    def on_close(self):
        self.connections.remove(self)

class FlightWSHandler(tornado.websocket.WebSocketHandler):
    connections = set()
    def check_origin(self, origin):
        return True

    def open(self):
        self.connections.add(self)

    def on_message(self, message):
        pass

    def on_close(self):
        self.connections.remove(self)

def make_app():
    return tornado.web.Application([
        (r"/", MainHandler),
        (r"/websocket", WSHandler),
        (r"/camera", CameraWSHandler),
        (r"/flight-control", FlightWSHandler),
        (r"/static/(.*)",tornado.web.StaticFileHandler, {"path": "./static"}),
    ],**settings)

if __name__ == "__main__":
    # If running on a Windows machine, use WindowsSelectorEventLoopPolicy
    if sys.platform == 'win32':
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
    # Initialize server, listen to port 8888
    app = make_app()
    app.listen(8888)
    tornado.ioloop.IOLoop.current().start()
