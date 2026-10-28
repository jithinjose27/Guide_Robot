#!/usr/bin/env python3
"""
G.L.A.D. Voice-Activated AI Assistant Node.

Manages conversational Q&A:
- Standby mode: Listens for wake words ("hey glad", "hello robo")
- Active mode: Captures user query, streams response from Google Gemini 2.5,
  and synthesizes speech in real-time via Microsoft Edge TTS / gTTS
- Sleep mode: Returns to low-power standby on sleep words ("thank you", "bye")
"""

import os
import re
import shutil
import subprocess
import tempfile
import threading
import time
import rclpy
from rclpy.node import Node

# Optional import of speech recognition
try:
    import speech_recognition as sr
except ImportError:
    sr = None


def find_usb_audio():
    """Locate USB microphone and speaker devices, or fallback to system defaults."""
    mic_idx = None
    if sr is not None:
        try:
            for i, name in enumerate(sr.Microphone.list_microphone_names()):
                if "YM USB Audio" in name or "USB Audio" in name:
                    mic_idx = i
                    break
        except Exception:
            pass

    speaker_hw = "default"
    try:
        out = subprocess.check_output(["aplay", "-l"], stderr=subprocess.DEVNULL).decode()
        for line in out.split("\n"):
            if "card" in line.lower() and ("YM USB Audio" in line or "USB Audio" in line):
                match = re.search(r"card (\d+)", line)
                if match:
                    speaker_hw = f"plughw:{match.group(1)},0"
                    break
    except Exception:
        pass

    return mic_idx, speaker_hw


DETECTED_MIC, DETECTED_SPEAKER = find_usb_audio()


class Config:
    ASSISTANT_NAME = "GLAD"
    MIC_INDEX = DETECTED_MIC
    SPEAKER_HW = DETECTED_SPEAKER

    GEMINI_API_KEY = os.getenv("GEMINI_API_KEY", "")
    GEMINI_MODEL_NAME = "gemini-2.5-flash-lite"

    WAKE_WORDS = [
        "hello robo", "hi robo", "hey robo",
        "hello glad", "hey glad", "hi glad",
        "hello dad", "hey dad", "hey lad", "hi lad",
    ]

    SLEEP_WORDS = [
        "thank you", "thanks", "stop glad", "go to sleep",
        "thank you robo", "bye", "thanks robo", "quiet",
    ]

    SYSTEM_INSTRUCTION = (
        f"You are {ASSISTANT_NAME}, a friendly and intelligent robotics laboratory assistant. "
        "Keep your responses strictly to 2 or 3 short conversational sentences. "
        "Be concise, warm, and helpful. Do not use markdown, asterisks, or bullet points."
    )


def init_gemini():
    """Initialize Google GenAI client if key is available."""
    if not Config.GEMINI_API_KEY or Config.GEMINI_API_KEY.startswith("YOUR_"):
        print("Note: GEMINI_API_KEY not set. Local Q&A fallback enabled.")
        return None
    try:
        from google import genai
        client = genai.Client(api_key=Config.GEMINI_API_KEY)
        print(f"[OK] Gemini AI ({Config.GEMINI_MODEL_NAME}) connected successfully.")
        return client
    except Exception as e:
        print(f"[WARN] Gemini client initialization error: {e}")
        return None


def ask_gemini_stream(client, prompt: str):
    """Yield sentences from Gemini streaming response or local fallback."""
    if not prompt:
        return

    if not client:
        # High quality local responses for common student inquiries
        p_lower = prompt.lower()
        if "who are you" in p_lower or "your name" in p_lower:
            yield "I am G.L.A.D, the Guide Lab Assistance Droid for the Robotics and Automation department."
        elif "kuka" in p_lower:
            yield "Our KUKA robot is a six-degree-of-freedom industrial manipulator used for precision kinematics and automation."
        elif "ros" in p_lower or "jazzy" in p_lower:
            yield "I am powered by ROS 2 Jazzy, using Nav2 for navigation and SLAM Toolbox for online laser mapping."
        else:
            yield "I am operating in local demonstration mode. Please set your Gemini API key to enable live AI reasoning."
        return

    try:
        from google.genai import types
        response_stream = client.models.generate_content_stream(
            model=Config.GEMINI_MODEL_NAME,
            contents=prompt,
            config=types.GenerateContentConfig(
                system_instruction=Config.SYSTEM_INSTRUCTION,
            ),
        )
        buffer = ""
        for chunk in response_stream:
            if chunk.text:
                buffer += chunk.text
                parts = re.split(r"([.?!:\n]+)", buffer)
                if len(parts) > 2:
                    for i in range(0, len(parts) - 1, 2):
                        sentence = (parts[i] + parts[i + 1]).strip()
                        if sentence:
                            yield sentence
                    buffer = parts[-1]

        if buffer.strip():
            yield buffer.strip()
    except Exception as e:
        error_msg = str(e)
        if "429" in error_msg or "RESOURCE_EXHAUSTED" in error_msg:
            yield "I am receiving too many requests right now. Please allow me one minute to reset."
        else:
            yield "I encountered a network issue contacting my knowledge base."


def speak(text: str, print_text: bool = True):
    """Synthesize speech using edge-tts or gTTS."""
    clean_text = text.replace("*", "").replace("#", "").strip()
    if not clean_text:
        return

    if print_text:
        print(f"{Config.ASSISTANT_NAME}: \"{clean_text}\"")

    try:
        with tempfile.NamedTemporaryFile(suffix=".mp3", delete=False) as tmp:
            temp_mp3 = tmp.name

        # 1. Prefer edge-tts if installed
        if shutil.which("edge-tts"):
            subprocess.run(
                [
                    "edge-tts",
                    "--voice", "en-US-AriaNeural",
                    "--text", clean_text,
                    "--write-media", temp_mp3,
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        else:
            from gtts import gTTS
            tts = gTTS(text=clean_text, lang="en", slow=False)
            tts.save(temp_mp3)

        # 2. Play audio
        if shutil.which("mpg123"):
            subprocess.run(
                ["mpg123", "-a", Config.SPEAKER_HW, "-q", temp_mp3],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        elif shutil.which("ffplay"):
            subprocess.run(
                ["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", temp_mp3],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )

        try:
            os.remove(temp_mp3)
        except Exception:
            pass
    except Exception as e:
        print(f"Audio playback note: {e}")


def listen_mic(status_message: str):
    """Capture audio from microphone and convert to text using speech recognition."""
    if sr is None:
        time.sleep(1.0)
        return None

    recognizer = sr.Recognizer()
    recognizer.dynamic_energy_threshold = False
    recognizer.energy_threshold = 1000
    recognizer.pause_threshold = 1.0

    try:
        with sr.Microphone(device_index=Config.MIC_INDEX) as source:
            print(f"\n{status_message}")
            audio = recognizer.listen(source, timeout=7, phrase_time_limit=15)
            text = recognizer.recognize_google(audio)
            print(f"User: \"{text}\"")
            return text.lower()
    except (sr.WaitTimeoutError, sr.UnknownValueError):
        return None
    except Exception as e:
        print(f"Microphone notice: {e}")
        return None


class VoiceAssistantNode(Node):
    def __init__(self):
        super().__init__("voice_assistant_node")
        self.get_logger().info(f"Starting {Config.ASSISTANT_NAME} Voice Q&A Assistant")

        self.client = init_gemini()
        self.is_awake = False
        self.running = True

        speak("Hello, I am glad, nice to meet you. Do you have any questions about the robotics lab?")

        # Run the listening loop in a background daemon thread so ROS 2 node can spin freely
        self.loop_thread = threading.Thread(target=self.run_voice_loop, daemon=True)
        self.loop_thread.start()

    def run_voice_loop(self):
        """Infinite loop with Standby / Active state management."""
        while self.running and rclpy.ok():
            try:
                prompt_msg = (
                    "Listening for a question..."
                    if self.is_awake
                    else "Waiting for wake word ('hey glad', 'hello robo')..."
                )
                command = listen_mic(prompt_msg)
            except Exception:
                time.sleep(0.5)
                continue

            if not command:
                continue

            if "shut down system" in command:
                speak("Shutting down the voice assistant. Goodbye.")
                self.running = False
                break

            # --- STANDBY MODE (Waiting for wake word) ---
            if not self.is_awake:
                if any(w in command for w in Config.WAKE_WORDS):
                    self.is_awake = True
                    speak("I am listening. How can I help you?")
                continue

            # --- ACTIVE MODE (Conversing) ---
            else:
                if any(s in command for s in Config.SLEEP_WORDS):
                    self.is_awake = False
                    speak("You are welcome! Going back to standby.")
                    continue

                clean_command = command
                for w in Config.WAKE_WORDS:
                    clean_command = clean_command.replace(w, "").strip()

                if clean_command:
                    print(f"{Config.ASSISTANT_NAME}: ", end="", flush=True)
                    for sentence in ask_gemini_stream(self.client, clean_command):
                        print(sentence + " ", end="", flush=True)
                        speak(sentence, print_text=False)
                    print()

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceAssistantNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
