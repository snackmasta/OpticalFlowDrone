import cv2
import subprocess
import time
from datetime import datetime
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent
RECORDINGS_DIR = Path("recordings")
MEDIAMTX_BIN = PROJECT_ROOT/ "./.." / ".tools" / "mediamtx" / "mediamtx"


def choose_output_mode():
    """
    Prompts the user via console input to select the output mode (either RTSP stream or local recording).
    """
    default_mode = "record"
    prompt = "Choose output mode: [r]ecord locally or [s]tream via RTSP? [r/s] "
    try:
        choice = input(prompt).strip().lower()
    except EOFError:
        choice = default_mode

    if choice in ("s", "stream", "rtsp"):
        return "rtsp"
    return default_mode


def choose_rtsp_url():
    """
    Prompts the user via console input to enter the RTSP stream name, falling back to 'drone'.
    """
    default_url = "drone"
    try:
        value = input(f"RTSP stream name [{default_url}]: ").strip()
    except EOFError:
        value = ""
    return value or default_url


class FileSink:
    """
    Saves video frames locally into an MP4 file.
    """
    def __init__(self, frame_size, fps):
        RECORDINGS_DIR.mkdir(exist_ok=True)
        self.output_path = RECORDINGS_DIR / f"optical_flow_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.writer = cv2.VideoWriter(str(self.output_path), fourcc, fps, frame_size)

        if not self.writer.isOpened():
            raise RuntimeError(f"Failed to open video writer for {self.output_path}")

        print(f"Recording optical flow to {self.output_path}")

    def write(self, frame):
        """
        Writes a single frame to the video file.
        """
        self.writer.write(frame)

    def release(self):
        """
        Releases the VideoWriter resource and prints the path of the saved video.
        """
        self.writer.release()
        print(f"Saved recording: {self.output_path}")


class RtspServer:
    """
    Spawns and manages a local MediaMTX RTSP server subprocess if one is not already running.
    """
    def __init__(self):
        if not MEDIAMTX_BIN.exists():
            raise RuntimeError(f"MediaMTX binary not found at {MEDIAMTX_BIN}")

        self.spawned = False
        self.process = None

        try:
            res = subprocess.run(["pgrep", "-f", "mediamtx"], capture_output=True)
            if res.returncode == 0:
                # mediamtx is already running
                return
        except Exception:
            pass

        log_dir = Path("logs")
        log_dir.mkdir(exist_ok=True)
        log_file = open(log_dir / "mediamtx.log", "a")

        self.process = subprocess.Popen(
            [str(MEDIAMTX_BIN)],
            cwd=str(MEDIAMTX_BIN.parent),
            stdout=log_file,
            stderr=log_file,
        )
        self.spawned = True

        time.sleep(1)
        if self.process.poll() is not None:
            raise RuntimeError("MediaMTX failed to start. Check logs/mediamtx.log for details.")

    def release(self):
        """
        Terminates the MediaMTX subprocess if spawned by this class.
        """
        if self.spawned and self.process and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except Exception:
                self.process.kill()


class RtspSink:
    """
    Streams video frames to a local RTSP server using ffmpeg via stdin pipe.
    """
    def __init__(self, frame_size, fps, rtsp_url):
        frame_width, frame_height = frame_size
        self.rtsp_url = rtsp_url
        self.failed = False
        self.process = subprocess.Popen(
            [
                "ffmpeg",
                "-loglevel",
                "error",
                "-f",
                "rawvideo",
                "-pix_fmt",
                "bgr24",
                "-s",
                f"{frame_width}x{frame_height}",
                "-r",
                str(fps),
                "-i",
                "-",
                "-an",
                "-c:v",
                "libx264",
                "-preset",
                "ultrafast",
                "-tune",
                "zerolatency",
                "-bf",
                "0",
                "-g",
                str(int(fps)),
                "-flags",
                "+global_header",
                "-bsf:v",
                "h264_mp4toannexb",
                "-pix_fmt",
                "yuv420p",
                "-rtsp_transport",
                "tcp",
                "-f",
                "rtsp",
                rtsp_url,
            ],
            stdin=subprocess.PIPE,
        )

        print(f"Streaming optical flow to {rtsp_url}")

    def write(self, frame):
        """
        Pipes a raw video frame to the running ffmpeg subprocess.
        """
        if self.failed:
            return
        if self.process.stdin is None:
            self.failed = True
            raise RuntimeError("RTSP stream is not writable")

        if self.process.poll() is not None:
            self.failed = True
            raise RuntimeError(f"RTSP server stopped for {self.rtsp_url}")

        try:
            self.process.stdin.write(frame.tobytes())
        except BrokenPipeError as exc:
            self.failed = True
            raise RuntimeError(f"RTSP stream dropped for {self.rtsp_url}") from exc
        except Exception:
            self.failed = True
            raise

    def is_failed(self):
        """
        Returns True if streaming has failed due to a broken pipe or connection drop.
        """
        return self.failed

    def release(self):
        """
        Closes the stdin stream to ffmpeg and waits for the subprocess to terminate.
        """
        if self.process.stdin is not None:
            try:
                self.process.stdin.close()
            except Exception:
                pass

        try:
            self.process.wait(timeout=5)
        except Exception:
            self.process.kill()


def create_output_sink(frame_size, fps, mode=None, rtsp_name=None):
    """
    Creates and returns the appropriate video output sink (FileSink or RtspSink) based on configuration.
    """
    if mode is None:
        mode = choose_output_mode()
    if mode == "rtsp":
        if rtsp_name is None:
            stream_name = choose_rtsp_url()
        else:
            stream_name = rtsp_name
        server = RtspServer()
        publish_url = f"rtsp://127.0.0.1:8554/{stream_name}"
        print(f"Open this on Windows: rtsp://<drone-ip>:8554/{stream_name}")
        return RtspSink(frame_size, fps, publish_url), True, server
    return FileSink(frame_size, fps), False, None
