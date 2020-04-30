import tornado.ioloop
import tornado.web
import tornado.websocket
import base64
import sys
import asyncio

class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("index.html")

class SimpleWebSocket(tornado.websocket.WebSocketHandler):
    connections = set()
    def check_origin(self, origin):
        return True

    def open(self):
        self.connections.add(self)

    def on_message(self, message):
        [client.write_message(message) for client in self.connections]
        print(message)
        #base64Message = base64.b64encode(message)
        #[client.write_message(base64Message) for client in self.connections]
    def on_close(self):
        self.connections.remove(self)

def make_app():
    return tornado.web.Application([
        (r"/", MainHandler),
        (r"/websocket", SimpleWebSocket)
    ])

if __name__ == "__main__":
    if sys.platform == 'win32':
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
    app = make_app()
    app.listen(8888)
    tornado.ioloop.IOLoop.current().start()
