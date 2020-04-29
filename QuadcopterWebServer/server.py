import tornado.ioloop
import tornado.web
import tornado.websocket
from PIL import Image
import os

class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.render('index.html')

    def post(self):
        data = self.request.body
        print("Image received")
        result_file = 'test'
        with open(result_file, 'wb') as file_handler:
            file_handler.write(data)
            Image.open(result_file).save(result_file + '.jpg', 'JPEG')
        os.remove(result_file)
        print("Image saved")

class MyStaticFileHandler(tornado.web.StaticFileHandler):
    def set_extra_headers(self, path):
        # Disable cache
        self.set_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')

def make_app():
    return tornado.web.Application([
        (r"/", MainHandler),
        (r"/(.*)",MyStaticFileHandler, {"path": "./"},),
    ])

if __name__ == "__main__":
    app = make_app()
    app.listen(8888)
    tornado.ioloop.IOLoop.current().start()
