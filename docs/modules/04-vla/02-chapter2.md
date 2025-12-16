# Chapter 2: Building a Voice-Controlled Robot with ROS 2 and Whisper

## 2.1 System Architecture: The Flow of Voice to Action

In this chapter, we will build a complete system that allows us to control a robot using natural language voice commands. This is a classic robotics project that provides a fantastic introduction to multimodal interaction. Our system will be composed of three distinct, decoupled ROS 2 nodes. This modular architecture is a core principle of ROS and makes the system easier to debug, maintain, and expand.

1.  **Audio Capture Node (`audio_capture_node.py`):** This node's sole responsibility is to interface with the microphone hardware. It captures raw audio data and publishes it in a continuous stream onto a ROS 2 topic.
2.  **Speech-to-Text Node (`whisper_service_node.py`):** This node acts as the "ears" of our robot. It subscribes to the raw audio stream. To avoid sending a constant, computationally expensive stream of data to the cloud, it will implement voice activity detection (VAD) to identify when a user is actually speaking. Once speech is detected and has concluded, it will take the audio segment, send it to the OpenAI Whisper API for transcription, and publish the resulting text to a new topic.
3.  **Command Interpretation Node (`command_node.py`):** This node is the "brain." It subscribes to the transcribed text topic. It then parses this text to understand the user's intent and translates that intent into a specific robot command (e.g., a `geometry_msgs/Twist` message) which it publishes to the robot's control topic (e.g., `/cmd_vel`).

***
*Placeholder for a detailed diagram: "Voice Control System Architecture." This diagram should clearly show the three nodes and the data flowing between them:
1.  A microphone icon points to the `Audio Capture Node`.
2.  The `Audio Capture Node` publishes an `audio_msgs/Audio` message to an `/audio` topic.
3.  The `Whisper Service Node` subscribes to the `/audio` topic. An arrow shows it communicating with a cloud icon labeled "OpenAI Whisper API".
4.  The `Whisper Service Node` then publishes a `std_msgs/String` message to a `/transcribed_text` topic.
5.  The `Command Node` subscribes to the `/transcribed_text` topic.
6.  Finally, the `Command Node` publishes a `geometry_msgs/Twist` message to the `/cmd_vel` topic, which points to a robot icon.
This visualization makes the data pipeline clear.
***

## 2.2 The Ears: Capturing Audio in ROS 2

First, we need to get audio from a microphone into our ROS 2 system. We'll use the `sounddevice` Python library, which provides a simple interface to the PortAudio library for cross-platform audio I/O.

**Prerequisites:**

```bash
# Install the sounddevice library and a simple VAD tool
pip install sounddevice webrtcvad

# Ensure you have a microphone connected and configured on your system.
```

**A More Robust `audio_capture_node.py`:**

This version publishes audio in a custom message type, which is better practice than using a generic `Int16MultiArray`.

```python
# In your ROS 2 package, create a 'msg' directory.
# Inside, create a file named 'Audio.msg' with the following content:
#
# std_msgs/Header header
# int16[] data
# uint32 sample_rate
#
# Remember to update package.xml and CMakeLists.txt to build this message!

# The node itself:
import rclpy
from rclpy.node import Node
from your_package_name.msg import Audio # Import the custom message
import sounddevice as sd
import numpy as np

class AudioCaptureNode(Node):
    def __init__(self):
        super().__init__('audio_capture_node')
        self.publisher_ = self.create_publisher(Audio, 'audio', 10)
        
        # Audio parameters
        self.sample_rate = 16000  # 16kHz is standard for speech recognition
        self.channels = 1
        self.block_size = int(self.sample_rate * 0.1) # 100ms chunks

        self.get_logger().info("Starting audio stream...")
        self.stream = sd.InputStream(
            callback=self.audio_callback,
            samplerate=self.sample_rate,
            channels=self.channels,
            blocksize=self.block_size,
            dtype='int16'
        )
        self.stream.start()
        self.get_logger().info("Audio stream started.")

    def audio_callback(self, indata, frames, time, status):
        if status:
            self.get_logger().warn(f"Audio callback status: {status}")
        
        msg = Audio()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "microphone"
        msg.data = indata.flatten().tolist()
        msg.sample_rate = self.sample_rate
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    audio_capture_node = AudioCaptureNode()
    try:
        rclpy.spin(audio_capture_node)
    except KeyboardInterrupt:
        pass
    finally:
        audio_capture_node.get_logger().info("Stopping audio stream.")
        audio_capture_node.stream.stop()
        audio_capture_node.stream.close()
        audio_capture_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```
*This improved node uses an `InputStream` for a more efficient, callback-based approach and publishes audio with header information and the sample rate, which is crucial for downstream nodes.*

## 2.3 The Translator: Speech-to-Text with Whisper

This node is the most complex. It will listen for audio, use a Voice Activity Detector (VAD) to decide when speech is present, buffer that speech, and send it for transcription.

**The `whisper_service_node.py` with Voice Activity Detection:**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from your_package_name.msg import Audio # The custom message
import openai
import numpy as np
import tempfile
import os
from scipy.io.wavfile import write
import webrtcvad

class WhisperServiceNode(Node):
    def __init__(self):
        super().__init__('whisper_service_node')
        self.subscription = self.create_subscription(
            Audio, 'audio', self.audio_callback, 10)
        self.publisher_ = self.create_publisher(String, 'transcribed_text', 10)
        
        # VAD setup
        self.vad = webrtcvad.Vad(3) # Aggressiveness mode 3 is the most aggressive
        self.speech_buffer = bytearray()
        self.is_speaking = False
        self.silence_frames = 0
        self.silence_threshold = 15 # Number of non-speech frames to end an utterance (1.5s)

    def audio_callback(self, msg: Audio):
        # VAD requires 16-bit PCM audio in 10, 20, or 30ms frames
        frame_duration_ms = 1000 * len(msg.data) / msg.sample_rate
        if frame_duration_ms not in [10.0, 20.0, 30.0, 100.0]: # Our node sends 100ms
             # This part is simplified; a real implementation would need to chunk the audio correctly for VAD.
             # For this example, we'll assume the audio_capture_node is configured to send 30ms chunks.
             pass

        audio_bytes = np.array(msg.data, dtype=np.int16).tobytes()
        is_speech = self.vad.is_speech(audio_bytes, msg.sample_rate)

        if self.is_speaking:
            self.speech_buffer.extend(audio_bytes)
            if not is_speech:
                self.silence_frames += 1
                if self.silence_frames > self.silence_threshold:
                    self.is_speaking = False
                    self.transcribe_buffer()
                    self.speech_buffer = bytearray()
            else:
                self.silence_frames = 0 # Reset silence counter
        elif is_speech:
            self.get_logger().info("Speech detected, recording...")
            self.is_speaking = True
            self.speech_buffer.extend(audio_bytes)
            self.silence_frames = 0

    def transcribe_buffer(self):
        self.get_logger().info("End of speech detected, transcribing...")
        if not self.speech_buffer:
            return

        audio_np = np.frombuffer(self.speech_buffer, dtype=np.int16)

        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp_audio_file:
            write(tmp_audio_file.name, self.subscription.msg_type.sample_rate, audio_np)
            tmp_audio_file.close()

            try:
                with open(tmp_audio_file.name, "rb") as f:
                    transcript = openai.Audio.transcribe("whisper-1", f)
                
                text = transcript['text']
                if text.strip(): # Only publish if there is non-empty text
                    text_msg = String()
                    text_msg.data = text
                    self.publisher_.publish(text_msg)
                    self.get_logger().info(f'Transcribed and published: "{text}"')
            except openai.APIError as e:
                self.get_logger().error(f"OpenAI API error during transcription: {e}")
            finally:
                os.remove(tmp_audio_file.name)

# main function as before...
```
*This version is much more practical. It uses VAD to avoid transcribing silence, making it more efficient and cost-effective.*

## 2.4 The Brain: From Text to Robot Control

Finally, the command node interprets the text. For now, we'll stick to simple keyword spotting, but this node is where you would integrate more advanced natural language understanding (NLU) in the future.

**The `command_node.py` script:**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

class CommandNode(Node):
    def __init__(self):
        super().__init__('command_node')
        self.subscription = self.create_subscription(
            String, 'transcribed_text', self.text_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info("Command node ready to receive text.")

    def text_callback(self, msg):
        command = msg.data.lower().strip()
        self.get_logger().info(f'Interpreting command: "{command}"')
        
        twist_msg = Twist()
        publish_duration = 2.0 # Publish the command for 2 seconds

        if "forward" in command:
            twist_msg.linear.x = 0.5
        elif "backward" in command or "back" in command:
            twist_msg.linear.x = -0.5
        elif "turn left" in command:
            twist_msg.angular.z = 0.5
        elif "turn right" in command:
            twist_msg.angular.z = -0.5
        elif "stop" in command:
            # Publish a zero-velocity message immediately
            self.cmd_vel_pub.publish(Twist())
            return
        else:
            self.get_logger().warn(f"Command not understood: '{command}'")
            return

        # Publish the command for a fixed duration
        self.get_logger().info(f"Executing command for {publish_duration} seconds.")
        start_time = time.time()
        while time.time() - start_time < publish_duration:
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(0.1)
        
        # Stop the robot after the duration
        self.cmd_vel_pub.publish(Twist())

# main function as before...
```
*This improved control node publishes the command for a set duration and then stops the robot, which is a more predictable behavior than a single, instantaneous publish.*

## 2.5 Summary

In this chapter, you learned how to architect and build a complete, modular, voice-controlled robotics system in ROS 2. You created a robust audio capture node, a sophisticated speech-to-text node using the OpenAI Whisper API with voice activity detection, and a command interpretation node to control the robot. This forms a powerful foundation for human-robot interaction. In the final chapter, we will replace the simple command interpreter with a much more powerful LLM-based planner, enabling the robot to understand complex, multi-step instructions.
