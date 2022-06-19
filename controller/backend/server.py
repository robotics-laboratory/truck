from fastapi import FastAPI, WebSocket
from starlette.responses import FileResponse
from backend.common import Const, Config, State
from backend.hardware import Hardware


class Server:
    def __init__(self):
        self.config = Config.from_yaml()
        self.state = State()
        self.hardware = Hardware(self.config, self.state)

    async def handle_ws(self, ws):
        pass


app = FastAPI()

@app.get("/")
async def index():
    return FileResponse("frontend/index.html")

@app.get("/api/test")
async def test():
    return "pong"
