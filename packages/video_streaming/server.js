const WebSocketServer = require("ws").Server;
const Splitter = require("stream-split");
const http = require('http');
const express = require('express');
const spawn = require('child_process').spawn;

const NALseparator = Buffer.from([0, 0, 0, 1]);

class StreamingServer {
    constructor(server, options) {
        this.server = new WebSocketServer({ server });
        this.server.on("connection", this.onConnect.bind(this));
        this.server.on("close", this.onClose.bind(this));
        this.resolution = options.resolution.split("x").map((x) => parseInt(x));
        this.pipelineCommand = this.buildPipelineCommand(options);
        this.process = null;
        this.socket = null;
    }

    buildPipelineCommand(options) {
        const stream = [
            "video/x-raw",
            "format=YUY2",
            `width=${options.resolution.split("x")[0]}`,
            `height=${options.resolution.split("x")[1]}`,
            `framerate=${options.framerate}/1`,
        ];
        const args = [
            "gst-launch-1.0", "v4l2src", `device=${options.device}`, "!",
            stream.join(", "), "!", "nvvidconv", "!",
            "nvv4l2h264enc", `bitrate=${options.bitrate}`,
            "preset-level=1", "maxperf-enable=true", "!",
            "fdsink", "sync=false"
        ];
        console.error("Video pipeline cmd:", args.join(" "));
        return args;
    }

    onConnect(socket) {
        console.error("New connection");
        if (this.socket) {
            console.error("Closing previous connection");
            this.socket.removeAllListeners();
            this.socket.close();
            this.process.kill();
        }
        this.socket = socket;

        socket.on("close", () => {
            console.error("Connection closed");
            this.socket.removeAllListeners();
            this.process.kill();
        });

        this.socket.send(JSON.stringify({
            action: "init",
            width: this.resolution[0],
            height: this.resolution[1],
        }));

        this.restartPipeline();
    }

    onClose() {
        console.error("Server closed");
        if (this.process) this.process.kill();
    }

    restartPipeline() {
        console.error("Restarting pipeline...");
        if (this.process) this.process.kill();
        this.process = spawn(this.pipelineCommand[0], this.pipelineCommand.slice(1));
        this.process.stderr.pipe(process.stderr);
        const splitter = new Splitter(NALseparator);
        const chunkStream = this.process.stdout.pipe(splitter);
        chunkStream.on("data", this.sendChunk.bind(this));
    }

    sendChunk(data) {
        if (!this.socket) return;
        this.socket.send(Buffer.concat([NALseparator, data]), { binary: true });
    }
}

const port = parseInt(process.env.STREAMING_PORT || "5555");
const device = process.env.STREAMING_DEVICE || "/dev/video4";
const resolution = process.env.STREAMING_RESOLUTION || "640x480";
const framerate = process.env.STREAMING_FRAMERATE || "15";
const bitrate = process.env.STREAMING_BITRATE || "600000";

const app = express();
const server = http.createServer(app);
const pipelineOptions = { device, resolution, framerate, bitrate };
const _ = new StreamingServer(server, pipelineOptions);
server.listen(port);
