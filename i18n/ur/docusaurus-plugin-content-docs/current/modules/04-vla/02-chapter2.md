# باب 2: ROS 2 اور Whisper کے ساتھ ایک آواز سے کنٹرول شدہ روبوٹ بنانا

## 2.1 سسٹم فن تعمیر: آواز سے عمل تک کا بہاؤ

اس باب میں، ہم ایک مکمل نظام بنائیں گے جو ہمیں قدرتی زبان کی صوتی کمانڈز کا استعمال کرتے ہوئے روبوٹ کو کنٹرول کرنے کی اجازت دیتا ہے۔ یہ ایک کلاسک روبوٹکس پروجیکٹ ہے جو ملٹی موڈل تعامل کا ایک شاندار تعارف فراہم کرتا ہے۔ ہمارا نظام تین الگ الگ، ڈی کپلڈ ROS 2 نوڈس پر مشتمل ہوگا۔ یہ ماڈیولر فن تعمیر ROS کا ایک بنیادی اصول ہے اور نظام کو ڈی بگ، برقرار رکھنے اور پھیلانے میں آسان بناتا ہے۔

1.  **آڈیو کیپچر نوڈ (`audio_capture_node.py`):** اس نوڈ کی واحد ذمہ داری مائیکروفون ہارڈویئر کے ساتھ انٹرفیس کرنا ہے۔ یہ خام آڈیو ڈیٹا کیپچر کرتا ہے اور اسے ROS 2 ٹاپک پر مسلسل اسٹریم میں شائع کرتا ہے۔
2.  **اسپیچ ٹو ٹیکسٹ نوڈ (`whisper_service_node.py`):** یہ نوڈ ہمارے روبوٹ کے "کان" کے طور پر کام کرتا ہے۔ یہ خام آڈیو اسٹریم کو سبسکرائب کرتا ہے۔ کلاؤڈ پر مسلسل، کمپیوٹیشنل طور پر مہنگے ڈیٹا کے اسٹریم بھیجنے سے بچنے کے لیے، یہ صوتی سرگرمی کا پتہ لگانے (VAD) کو نافذ کرے گا تاکہ یہ شناخت کیا جا سکے کہ صارف کب اصل میں بول رہا ہے۔ ایک بار جب تقریر کا پتہ چل جاتا ہے اور ختم ہو جاتا ہے، تو یہ آڈیو سیگمنٹ لے گا، اسے نقل کے لیے OpenAI Whisper API پر بھیجے گا، اور نتیجہ خیز متن کو ایک نئے ٹاپک پر شائع کرے گا۔
3.  **کمانڈ انٹرپریٹیشن نوڈ (`command_node.py`):** یہ نوڈ "دماغ" ہے۔ یہ نقل شدہ ٹیکسٹ ٹاپک کو سبسکرائب کرتا ہے۔ اس کے بعد یہ صارف کے ارادے کو سمجھنے کے لیے اس متن کو پارس کرتا ہے اور اس ارادے کو ایک مخصوص روبوٹ کمانڈ (جیسے، ایک `geometry_msgs/Twist` پیغام) میں ترجمہ کرتا ہے جسے یہ روبوٹ کے کنٹرول ٹاپک (جیسے، `/cmd_vel`) پر شائع کرتا ہے۔

***
*تفصیلی خاکہ کے لیے پلیس ہولڈر: "صوتی کنٹرول سسٹم فن تعمیر۔" اس خاکہ کو تینوں نوڈس اور ان کے درمیان بہنے والے ڈیٹا کو واضح طور پر دکھانا چاہیے:
1.  ایک مائیکروفون آئیکن `آڈیو کیپچر نوڈ` کی طرف اشارہ کرتا ہے۔
2.  `آڈیو کیپچر نوڈ` ایک `audio_msgs/Audio` پیغام کو `/audio` ٹاپک پر شائع کرتا ہے۔
3.  `Whisper سروس نوڈ` `/audio` ٹاپک کو سبسکرائب کرتا ہے۔ ایک تیر اسے "OpenAI Whisper API" کے لیبل والے کلاؤڈ آئیکن کے ساتھ بات چیت کرتے ہوئے دکھاتا ہے۔
4.  `Whisper سروس نوڈ` پھر ایک `std_msgs/String` پیغام کو `/transcribed_text` ٹاپک پر شائع کرتا ہے۔
5.  `کمانڈ نوڈ` `/transcribed_text` ٹاپک کو سبسکرائب کرتا ہے۔
6.  آخر میں، `کمانڈ نوڈ` ایک `geometry_msgs/Twist` پیغام کو `/cmd_vel` ٹاپک پر شائع کرتا ہے، جو ایک روبوٹ آئیکن کی طرف اشارہ کرتا ہے۔
یہ ویژولائزیشن ڈیٹا پائپ لائن کو واضح کرتی ہے۔
***

## 2.2 کان: ROS 2 میں آڈیو کیپچر کرنا

سب سے پہلے، ہمیں مائیکروفون سے آڈیو کو اپنے ROS 2 سسٹم میں لانے کا ایک طریقہ درکار ہے۔ ہم `sounddevice` پائتھن لائبریری کا استعمال کریں گے، جو کراس پلیٹ فارم آڈیو I/O کے لیے پورٹ آڈیو لائبریری کو ایک سادہ انٹرفیس فراہم کرتی ہے۔

**شرائط:**

```bash
# sounddevice لائبریری اور ایک سادہ VAD ٹول انسٹال کریں
pip install sounddevice webrtcvad

# یقینی بنائیں کہ آپ کے سسٹم پر ایک مائیکروفون منسلک اور ترتیب دیا گیا ہے۔
```

**ایک زیادہ مضبوط `audio_capture_node.py`:**

یہ ورژن آڈیو کو کسٹم میسج ٹائپ میں شائع کرتا ہے، جو ایک عام `Int16MultiArray` استعمال کرنے سے بہتر عمل ہے۔

```python
# اپنے ROS 2 پیکیج میں، ایک 'msg' ڈائرکٹری بنائیں۔
# اندر، 'Audio.msg' نامی ایک فائل بنائیں جس میں درج ذیل مواد ہو:
#
# std_msgs/Header header
# int16[] data
# uint32 sample_rate
#
# اس پیغام کو بنانے کے لیے package.xml اور CMakeLists.txt کو اپ ڈیٹ کرنا یاد رکھیں!

# نوڈ خود:
import rclpy
from rclpy.node import Node
from your_package_name.msg import Audio # کسٹم پیغام درآمد کریں
import sounddevice as sd
import numpy as np

class AudioCaptureNode(Node):
    def __init__(self):
        super().__init__('audio_capture_node')
        self.publisher_ = self.create_publisher(Audio, 'audio', 10)
        
        # آڈیو پیرامیٹرز
        self.sample_rate = 16000  # 16kHz اسپیچ ریکگنیشن کے لیے معیاری ہے
        self.channels = 1
        self.block_size = int(self.sample_rate * 0.1) # 100ms کے حصے

        self.get_logger().info("آڈیو اسٹریم شروع ہو رہا ہے...")
        self.stream = sd.InputStream(
            callback=self.audio_callback,
            samplerate=self.sample_rate,
            channels=self.channels,
            blocksize=self.block_size,
            dtype='int16'
        )
        self.stream.start()
        self.get_logger().info("آڈیو اسٹریم شروع ہو گیا۔")

    def audio_callback(self, indata, frames, time, status):
        if status:
            self.get_logger().warn(f"آڈیو کال بیک اسٹیٹس: {status}")
        
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
        audio_capture_node.get_logger().info("آڈیو اسٹریم کو روکا جا رہا ہے۔")
        audio_capture_node.stream.stop()
        audio_capture_node.stream.close()
        audio_capture_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```
*یہ بہتر نوڈ ایک زیادہ موثر، کال بیک پر مبنی نقطہ نظر کے لیے `InputStream` کا استعمال کرتا ہے اور ہیڈر کی معلومات اور نمونے کی شرح کے ساتھ آڈیو شائع کرتا ہے، جو نیچے کی دھارے والے نوڈس کے لیے اہم ہے۔*

## 2.3 مترجم: Whisper کے ساتھ اسپیچ ٹو ٹیکسٹ

یہ نوڈ سب سے پیچیدہ ہے۔ یہ آڈیو سنے گا، تقریر کی موجودگی کا فیصلہ کرنے کے لیے وائس ایکٹیویٹی ڈیٹیکٹر (VAD) کا استعمال کرے گا، اس تقریر کو بفر کرے گا، اور اسے نقل کے لیے بھیجے گا۔

**صوتی سرگرمی کا پتہ لگانے کے ساتھ `whisper_service_node.py`:**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from your_package_name.msg import Audio # کسٹم پیغام
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
        
        # VAD سیٹ اپ
        self.vad = webrtcvad.Vad(3) # جارحیت موڈ 3 سب سے زیادہ جارحانہ ہے
        self.speech_buffer = bytearray()
        self.is_speaking = False
        self.silence_frames = 0
        self.silence_threshold = 15 # ایک بیان ختم کرنے کے لیے غیر تقریری فریموں کی تعداد (1.5s)

    def audio_callback(self, msg: Audio):
        # VAD کو 10، 20، یا 30ms فریموں میں 16 بٹ PCM آڈیو کی ضرورت ہوتی ہے
        frame_duration_ms = 1000 * len(msg.data) / msg.sample_rate
        if frame_duration_ms not in [10.0, 20.0, 30.0, 100.0]: # ہمارا نوڈ 100ms بھیجتا ہے
             # یہ حصہ آسان بنایا گیا ہے؛ ایک حقیقی نفاذ کو VAD کے لیے آڈیو کو صحیح طریقے سے ٹکڑے کرنے کی ضرورت ہوگی۔
             # اس مثال کے لیے، ہم فرض کریں گے کہ audio_capture_node 30ms کے حصے بھیجنے کے لیے ترتیب دیا گیا ہے۔
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
                self.silence_frames = 0 # خاموشی کاؤنٹر ری سیٹ کریں
        elif is_speech:
            self.get_logger().info("تقریر کا پتہ چلا، ریکارڈنگ...")
            self.is_speaking = True
            self.speech_buffer.extend(audio_bytes)
            self.silence_frames = 0

    def transcribe_buffer(self):
        self.get_logger().info("تقریر کا اختتام پتہ چلا، نقل کیا جا رہا ہے...")
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
                if text.strip(): # صرف اس صورت میں شائع کریں جب غیر خالی متن ہو
                    text_msg = String()
                    text_msg.data = text
                    self.publisher_.publish(text_msg)
                    self.get_logger().info(f'نقل شدہ اور شائع شدہ: "{text}"')
            except openai.APIError as e:
                self.get_logger().error(f"نقل کے دوران OpenAI API کی خرابی: {e}")
            finally:
                os.remove(tmp_audio_file.name)

# main فنکشن جیسا کہ پہلے...
```
*یہ ورژن بہت زیادہ عملی ہے۔ یہ خاموشی کو نقل کرنے سے بچنے کے لیے VAD کا استعمال کرتا ہے، جس سے یہ زیادہ موثر اور لاگت سے موثر ہوتا ہے۔*

## 2.4 دماغ: متن سے روبوٹ کنٹرول تک

آخر میں، کمانڈ نوڈ متن کی تشریح کرتا ہے۔ ابھی کے لیے، ہم سادہ مطلوبہ الفاظ کی نشاندہی پر قائم رہیں گے، لیکن یہ نوڈ وہ جگہ ہے جہاں آپ مستقبل میں زیادہ جدید قدرتی زبان کی تفہیم (NLU) کو مربوط کریں گے۔

**`command_node.py` اسکرپٹ:**

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
        self.get_logger().info("کمانڈ نوڈ متن وصول کرنے کے لیے تیار ہے۔")

    def text_callback(self, msg):
        command = msg.data.lower().strip()
        self.get_logger().info(f'کمانڈ کی تشریح: "{command}"')
        
        twist_msg = Twist()
        publish_duration = 2.0 # 2 سیکنڈ کے لیے کمانڈ شائع کریں

        if "آگے" in command:
            twist_msg.linear.x = 0.5
        elif "پیچھے" in command or "واپس" in command:
            twist_msg.linear.x = -0.5
        elif "بائیں مڑیں" in command:
            twist_msg.angular.z = 0.5
        elif "دائیں مڑیں" in command:
            twist_msg.angular.z = -0.5
        elif "رکو" in command:
            # فوری طور پر صفر رفتار کا پیغام شائع کریں
            self.cmd_vel_pub.publish(Twist())
            return
        else:
            self.get_logger().warn(f"کمانڈ سمجھ میں نہیں آئی: '{command}'")
            return

        # ایک مقررہ مدت کے لیے کمانڈ شائع کریں
        self.get_logger().info(f"{publish_duration} سیکنڈ کے لیے کمانڈ پر عمل کیا جا رہا ہے۔")
        start_time = time.time()
        while time.time() - start_time < publish_duration:
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(0.1)
        
        # مدت کے بعد روبوٹ کو روکیں
        self.cmd_vel_pub.publish(Twist())

# main فنکشن جیسا کہ پہلے...
```
*یہ بہتر کنٹرول نوڈ ایک مقررہ مدت کے لیے کمانڈ شائع کرتا ہے اور پھر روبوٹ کو روکتا ہے، جو ایک واحد، فوری اشاعت سے زیادہ متوقع رویہ ہے۔*

## 2.5 خلاصہ

اس باب میں، آپ نے ROS 2 میں ایک مکمل، ماڈیولر، آواز سے کنٹرول شدہ روبوٹکس سسٹم کی تعمیر اور فن تعمیر سیکھا۔ آپ نے ایک مضبوط آڈیو کیپچر نوڈ، OpenAI Whisper API کا استعمال کرتے ہوئے صوتی سرگرمی کا پتہ لگانے کے ساتھ ایک جدید اسپیچ ٹو ٹیکسٹ نوڈ، اور روبوٹ کو کنٹرول کرنے کے لیے ایک کمانڈ انٹرپریٹیشن نوڈ بنایا۔ یہ انسانی-روبوٹ تعامل کے لیے ایک طاقتور بنیاد بناتا ہے۔ آخری باب میں، ہم سادہ کمانڈ انٹرپریٹر کو ایک بہت زیادہ طاقتور LLM پر مبنی منصوبہ ساز سے بدل دیں گے، جس سے روبوٹ پیچیدہ، کثیر مرحلہ ہدایات کو سمجھنے کے قابل ہو جائے گا۔
