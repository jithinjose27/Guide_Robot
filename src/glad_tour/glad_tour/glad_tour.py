#!/usr/bin/env python3
"""
G.L.A.D. Autonomous Tour Guide Node.

Commands the mobile robot across predefined educational lab stations using
the Nav2 BasicNavigator API. Upon arrival at each station, synthesizes
location-aware spoken commentary using Google Gemini and Text-to-Speech (TTS).
Includes built-in curated commentary fallbacks if offline or no API key is provided.
"""

import os
import re
import shutil
import subprocess
import tempfile
import time
import math
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Pre-programmed high-quality tour scripts used when offline or without an API key
DEFAULT_STATION_SCRIPTS = {
    "Point 1": (
        "Welcome everyone to the Programming and Software Development Laboratory! "
        "Here, students learn robotics algorithms, develop autonomous control stacks in ROS 2, "
        "and simulate intelligent multi-robot systems."
    ),
    "Point 2": (
        "We have arrived at our premier Robotics Laboratory. "
        "Directly in front of you is our industrial KUKA robotic manipulator, "
        "utilized for precision trajectory planning, kinematic analysis, and advanced pick-and-place automation."
    ),
    "Point 3": (
        "This is our final stop: the Industrial Automation Laboratory. "
        "This facility is equipped with industrial electro-pneumatic and hydraulic workbenches "
        "where students master fluid power dynamics, PLC ladder logic, and sequential automated control."
    ),
}

GEMINI_API_KEY = os.getenv("GEMINI_API_KEY", "")
GEMINI_MODEL_NAME = "gemini-2.5-flash-lite"

SYSTEM_INSTRUCTION = (
    "You are G.L.A.D. (Guide Lab Assistance Droid), a friendly, intelligent, and "
    "professional autonomous tour guide robot in the Robotics and Automation department. "
    "Keep your response strictly to 2 or 3 short conversational sentences. "
    "Be warm, informative, and do not use any asterisks, markdown, or bullet points."
)


def find_usb_speaker() -> str:
    """Auto-detect USB audio card or fallback to standard audio sink."""
    try:
        out = subprocess.check_output(["aplay", "-l"], stderr=subprocess.DEVNULL).decode()
        for line in out.split("\n"):
            if "card" in line.lower() and ("YM USB Audio" in line or "USB Audio" in line):
                match = re.search(r"card (\d+)", line)
                if match:
                    return f"plughw:{match.group(1)},0"
    except Exception:
        pass
    return "default"


SPEAKER_HW = find_usb_speaker()


def init_gemini():
    """Initialize the Google GenAI SDK client if API key is present."""
    if not GEMINI_API_KEY or GEMINI_API_KEY.startswith("YOUR_"):
        print("Note: GEMINI_API_KEY not set. Using built-in laboratory tour scripts.")
        return None
    try:
        from google import genai
        client = genai.Client(api_key=GEMINI_API_KEY)
        print(f"[OK] Gemini AI ({GEMINI_MODEL_NAME}) initialized successfully.")
        return client
    except Exception as e:
        print(f"[WARN] Gemini client initialization notice: {e}. Falling back to local scripts.")
        return None


def generate_speech(client, station_name: str, prompt: str) -> str:
    """Generate dynamic speech with Gemini or use curated station script fallback."""
    if client:
        try:
            from google.genai import types
            response = client.models.generate_content(
                model=GEMINI_MODEL_NAME,
                contents=prompt,
                config=types.GenerateContentConfig(
                    system_instruction=SYSTEM_INSTRUCTION,
                    temperature=0.7,
                ),
            )
            text = response.text.strip()
            if text:
                return text
        except Exception as e:
            print(f"[WARN] Gemini API error ({e}). Using curated station script.")

    return DEFAULT_STATION_SCRIPTS.get(station_name, "Welcome to this department laboratory station!")


def speak(text: str):
    """Play speech aloud through USB speaker using gTTS and available audio player."""
    clean_text = text.replace("*", "").replace("#", "").strip()
    if not clean_text:
        return

    print(f"\nG.L.A.D.: \"{clean_text}\"\n")

    # Try generating speech audio via gTTS if installed
    try:
        from gtts import gTTS
        with tempfile.NamedTemporaryFile(suffix=".mp3", delete=False) as tmp:
            temp_mp3 = tmp.name

        tts = gTTS(text=clean_text, lang="en", slow=False)
        tts.save(temp_mp3)

        # Detect audio player
        if shutil.which("mpg123"):
            subprocess.run(
                ["mpg123", "-a", SPEAKER_HW, "-q", temp_mp3],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        elif shutil.which("ffplay"):
            subprocess.run(
                ["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", temp_mp3],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        elif shutil.which("paplay"):
            subprocess.run(["paplay", temp_mp3], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        try:
            os.remove(temp_mp3)
        except Exception:
            pass
    except Exception as e:
        # If gTTS or audio output fails, console output remains crystal clear
        print(f"Audio playback notice: {e}")


def make_quaternion(z: float, w: float):
    """Ensure a valid normalized 2D planar quaternion."""
    norm = math.sqrt(z * z + w * w)
    if norm > 1e-6:
        return z / norm, w / norm
    return 0.0, 1.0


def main(args=None):
    rclpy.init(args=args)

    print("==========================================")
    print("      G.L.A.D. AUTONOMOUS TOUR GUIDE      ")
    print("==========================================")

    gemini_client = init_gemini()
    navigator = BasicNavigator()

    print("Setting initial robot pose in map frame...")
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0

    navigator.setInitialPose(initial_pose)

    print("Waiting for Nav2 navigation stack to become active...")
    navigator.waitUntilNav2Active()
    print("[OK] Nav2 stack is fully active and ready.")

    speak("Navigation systems are online. Hello everyone, let us begin the tour!")

    waypoints = [
        {
            "name": "Point 1",
            "x": 3.44,
            "y": 0.595,
            "z": 0.937,
            "w": 0.348,
            "prompt": "We have just arrived at the programming Lab. Greet the guests and give a brief overview of the programming lab.",
        },
        {
            "name": "Point 2",
            "x": 3.17,
            "y": 3.66,
            "z": 0.975,
            "w": 0.221,
            "prompt": "Next, this is our main robotics Lab. Greet the guests and give a brief overview of the robotics lab and the KUKA industrial robot present inside.",
        },
        {
            "name": "Point 3",
            "x": 3.60,
            "y": 0.452,
            "z": 0.993,
            "w": 0.119,
            "prompt": "This is our final destination, the Automation Lab. Briefly explain the pneumatic and hydraulic systems we have in our automation lab.",
        },
    ]

    for point in waypoints:
        station_name = point["name"]
        print(f"\nNavigating to {station_name} (x={point['x']}, y={point['y']})...")

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(point["x"])
        goal_pose.pose.position.y = float(point["y"])
        goal_pose.pose.position.z = 0.0

        qz, qw = make_quaternion(point.get("z", 0.0), point.get("w", 1.0))
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw

        navigator.goToPose(goal_pose)

        # Non-blocking polling loop with sleep to prevent CPU starvation
        last_feedback_print = time.time()
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback and (time.time() - last_feedback_print > 4.0):
                dist = getattr(feedback, "distance_remaining", 0.0)
                print(f"  Distance remaining: {dist:.2f} m")
                last_feedback_print = time.time()
            time.sleep(0.1)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"[OK] Arrived successfully at {station_name}!")
            speech = generate_speech(gemini_client, station_name, point["prompt"])
            speak(speech)
            time.sleep(1.0)
        elif result == TaskResult.CANCELED:
            print(f"[WARN] Navigation to {station_name} was canceled.")
            speak("My navigation task was canceled.")
            break
        elif result == TaskResult.FAILED:
            print(f"[ERROR] Failed to reach {station_name}.")
            speak("I apologize, my path appears blocked. I will pause here.")
            break

    speak("That concludes our laboratory tour! Thank you for visiting, and feel free to ask questions.")
    print("Tour completed successfully. Shutting down navigator.")

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
