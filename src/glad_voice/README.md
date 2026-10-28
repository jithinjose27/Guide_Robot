# GLAD Voice (Conversational AI Assistant) 🎙️

Welcome to the **`glad_voice`** package! While navigation allows G.L.A.D to move around, the voice assistant gives the robot its personality and ability to engage in natural, interactive spoken Q&A with visitors and students.

---

## 🗣️ How Voice Interaction Works

```
           Microphone Audio (USB Sound Card)
                         │
                         ▼
┌────────────────────────────────────────────────────────┐
│                   1. Wake Word Filter                  │
│       Listens for: "hey glad", "hello robo", etc.      │
└────────────────────────┬───────────────────────────────┘
                         │ (Detected!)
                         ▼
┌────────────────────────────────────────────────────────┐
│             2. Speech Recognition (STT)                │
│    Converts visitor voice audio into clean text prompt │
└────────────────────────┬───────────────────────────────┘
                         │ User Query Text
                         ▼
┌────────────────────────────────────────────────────────┐
│         3. Generative AI Reasoning (LLM)               │
│   - Streams prompt to Google Gemini 2.5 Flash          │
│   - Answers dynamically in 2-3 concise sentences       │
│   - (Local script fallback if offline / no API key)    │
└────────────────────────┬───────────────────────────────┘
                         │ Streaming Sentences
                         ▼
┌────────────────────────────────────────────────────────┐
│             4. Text-to-Speech (TTS) Engine             │
│   - Microsoft Edge TTS (en-US-AriaNeural) or gTTS      │
│   - Plays through USB speaker via mpg123 / ffplay      │
└────────────────────────────────────────────────────────┘
```

---

## 🎯 State Machine: Standby vs. Active

1. **Standby Mode (Sleep):**
   - The robot keeps its mouth shut to prevent echoes and ignores casual ambient chatter.
   - It only wakes up when someone addresses it directly with a wake word:
     - `"hey glad"`, `"hello robo"`, `"hi glad"`, `"hey robo"`
2. **Active Mode (Awake):**
   - Responds: *"I am listening. How can I help you?"*
   - Listens for a question, strips the wake prefix, queries Gemini AI, and speaks the answer.
3. **Return to Standby:**
   - Say: `"thank you"`, `"thanks"`, `"go to sleep"`, or `"bye"`.
   - The robot replies politely and returns to standby.

---

## 🛠️ Hardware Requirements & Software Setup

### Required Hardware:
- Any standard USB Audio Dongle or USB Speakerphone with built-in microphone and 3.5mm/speaker output.

### Python Packages:
```bash
pip install SpeechRecognition google-genai edge-tts gTTS
sudo apt install mpg123 portaudio19-dev python3-pyaudio
```

### Environment Variable (Optional):
```bash
export GEMINI_API_KEY="AIzaSy..."
```
*(If no API key is exported, G.L.A.D still talks using local laboratory knowledge scripts!)*

---

## 🚀 Building & Launching

```bash
colcon build --packages-select glad_voice
source install/setup.bash
```

Run the voice assistant:
```bash
ros2 run glad_voice voice_assistant
```
