# باب 2: `rclpy` کے ساتھ ہیومنائڈز کو کنٹرول کرنا

## `rclpy` کے ساتھ پائتھن میں ROS 2 نوڈس لکھنا

**`rclpy`** ROS 2 کے لیے پائتھن کلائنٹ لائبریری ہے۔ یہ ROS 2 ایپلی کیشنز، بشمول نوڈس، پبلشرز، سبسکرائبرز، سروسز، اور ایکشنز لکھنے کا ایک آسان اور پائتھونک طریقہ فراہم کرتی ہے۔ اگر آپ پائتھن سے واقف ہیں، تو `rclpy` آپ کو روبوٹک فنکشنلٹیز کو تیزی سے تیار اور ٹیسٹ کرنے کی اجازت دیتا ہے۔

شروع کرنے سے پہلے، یقینی بنائیں کہ آپ نے ROS 2 انسٹال کر لیا ہے (ماڈیول 1) اور آپ کا پائتھن ماحول `rclpy` کے ساتھ سیٹ اپ ہے۔

### بنیادی `rclpy` نوڈ کی ساخت

ہر `rclpy` نوڈ `rclpy.node.Node` سے وراثت میں ملتا ہے۔ یہاں سب سے بنیادی ساخت ہے:

```python
import rclpy
from rclpy.node import Node

class MyCustomNode(Node):
    def __init__(self):
        super().__init__('my_custom_node') # نوڈ کو ایک منفرد نام کے ساتھ شروع کریں
        self.get_logger().info('میرا کسٹم نوڈ شروع ہو گیا ہے!')

def main(args=None):
    rclpy.init(args=args) # rclpy کو شروع کریں
    node = MyCustomNode() # نوڈ بنائیں
    try:
        rclpy.spin(node) # نوڈ کو زندہ رکھیں اور کال بیکس پر کارروائی کریں
    except KeyboardInterrupt:
        node.get_logger().info('نوڈ صاف طور پر رک گیا')
    finally:
        node.destroy_node() # نوڈ کو تباہ کریں
        rclpy.shutdown() # rclpy کو بند کریں

if __name__ == '__main__':
    main()
```
*   `rclpy.init(args=args)`: ROS 2 کلائنٹ لائبریری کو شروع کرتا ہے۔ کسی بھی ROS 2 سرگرمی سے پہلے اسے کال کرنا ضروری ہے۔
*   `super().__init__('my_custom_node')`: بیس `Node` کلاس کنسٹرکٹر کو کال کرتا ہے، نوڈ کو ایک نام دیتا ہے۔
*   `rclpy.spin(node)`: نوڈ بند ہونے تک بلاک کرتا ہے، جس سے کال بیکس (جیسے سبسکرائبرز، ٹائمرز سے) پر کارروائی کی جا سکتی ہے۔
*   `node.destroy_node()` اور `rclpy.shutdown()`: نوڈ اور کلائنٹ لائبریری کو صاف طور پر بند کرتا ہے۔

## پائتھن AI ایجنٹس کو ROS کنٹرولرز سے جوڑنا

`rclpy` پائتھن میں لکھے گئے AI ایجنٹس کو ROS 2 پر مبنی روبوٹ کنٹرولرز کے ساتھ بغیر کسی رکاوٹ کے جوڑنے کے قابل بناتا ہے۔ ایک AI ایجنٹ ایک کمک سیکھنے کی پالیسی، ایک کلاسیکی منصوبہ بندی الگورتھم، یا ایک LLM پر مبنی علمی منصوبہ ساز ہو سکتا ہے (جیسا کہ ہم ماڈیول 4 میں دیکھیں گے)۔ `rclpy` کا استعمال کرتے ہوئے، یہ ایجنٹ کر سکتے ہیں:

*   **سینسر ڈیٹا وصول کریں**: کیمرہ امیجز، LiDAR اسکینز، جوائنٹ اسٹیٹس وغیرہ شائع کرنے والے ٹاپکس کو سبسکرائب کریں۔
*   **کمانڈز بھیجیں**: ویلوسٹی کمانڈز، جوائنٹ پوزیشن اہداف شائع کریں، یا روبوٹ کنٹرولرز کو ROS 2 ایکشنز/سروسز کو ٹرگر کریں۔
*   **پرسیپشن/پلاننگ کے ساتھ مربوط ہوں**: دوسرے ROS 2 نوڈس (جیسے آبجیکٹ ڈیٹیکشنز، نیویگیشن پاتھ) کے آؤٹ پٹس کا استعمال کریں۔

## `rclpy` میں پبلشرز، سبسکرائبرز، ٹائمرز، اور سروسز کا استعمال

### پائتھن پبلشر مثال (`motor_command_publisher.py`)

یہ نوڈ ایک AI ایجنٹ کی نقالی کرے گا جو موٹر کمانڈز شائع کرتا ہے۔

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist # ویلوسٹی کمانڈز کے لیے عام پیغام

class MotorCommandPublisher(Node):
    def __init__(self):
        super().__init__('motor_command_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_period = 0.1 # ہر 100ms پر شائع کریں
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.get_logger().info('موٹر کمانڈ پبلشر شروع ہو گیا۔')

    def timer_callback(self):
        msg = Twist()
        self.linear_x = 0.1 # 0.1 m/s پر آگے بڑھیں
        self.angular_z = 0.0 # کوئی گردش نہیں
        msg.linear.x = self.linear_x
        msg.angular.z = self.angular_z
        self.publisher_.publish(msg)
        self.get_logger().info(f'ویلوسٹی شائع کی جا رہی ہے: linear_x={self.linear_x:.2f}, angular_z={self.angular_z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorCommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### پائتھن سبسکرائبر مثال (`joint_state_subscriber.py`)

یہ نوڈ روبوٹ سے نقلی جوائنٹ اسٹیٹس کو سبسکرائب کرے گا۔

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState # جوائنٹ اسٹیٹس کے لیے عام پیغام

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states', # 'joint_states' ٹاپک فرض کرتے ہوئے
            self.listener_callback,
            10)
        self.subscription # غیر استعمال شدہ متغیر انتباہ کو روکیں
        self.get_logger().info('جوائنٹ اسٹیٹ سبسکرائبر شروع ہو گیا۔')

    def listener_callback(self, msg):
        # self.get_logger().info(f'موصولہ جوائنٹ اسٹیٹس: {msg.name}, {msg.position}')
        for i in range(len(msg.name)):
            self.get_logger().info(f'  جوائنٹ: {msg.name[i]}, پوزیشن: {msg.position[i]:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = JointStateSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### پائتھن ٹائمر مثال (`status_reporter.py`)

ٹائمرز ایک نوڈ کو وقتاً فوقتاً کال بیک فنکشن پر عمل کرنے کی اجازت دیتے ہیں۔

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StatusReporter(Node):
    def __init__(self):
        super().__init__('status_reporter')
        self.publisher_ = self.create_publisher(String, 'robot_status', 10)
        self.timer_period = 2.0 # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('اسٹیٹس رپورٹر شروع ہو گیا۔')

    def timer_callback(self):
        msg = String()
        msg.data = 'روبوٹ آپریشنل ہے اور اسٹینڈ بائی پر ہے۔'
        self.publisher_.publish(msg)
        self.get_logger().info(f'اسٹیٹس شائع کیا جا رہا ہے: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = StatusReporter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### پائتھن سروس سرور/کلائنٹ مثال (`add_two_ints_server.py`, `add_two_ints_client.py`)

ROS 2 سروسز کو `.srv` فائل کی ضرورت ہوتی ہے جو درخواست اور جواب کی اقسام کی وضاحت کرتی ہے۔ اس مثال کے لیے، فرض کریں کہ آپ کے پاس `AddTwoInts.srv` کے ساتھ ایک کسٹم پیکیج ہے:

```
# AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

**سروس سرور:**

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # فرض کرتے ہوئے کہ یہ SRV دستیاب ہے

class AddTwoIntsService(Node):
    def __init__(self):
        super().__init__('add_two_ints_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('ایڈ ٹو انٹس سروس شروع ہو گئی۔')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'آنے والی درخواست: a={request.a}, b={request.b}')
        self.get_logger().info(f'جواب بھیجا جا رہا ہے: sum={response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**سروس کلائنٹ:**

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # فرض کرتے ہوئے کہ یہ SRV دستیاب ہے
import sys

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('سروس دستیاب نہیں، دوبارہ انتظار کر رہے ہیں...')
        self.req = AddTwoInts.Request()
        self.get_logger().info('ایڈ ٹو انٹس کلائنٹ شروع ہو گیا۔')

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 3:
        node = Node('add_two_ints_client_error')
        node.get_logger().error('استعمال: ros2 run <package_name> add_two_ints_client A B')
        rclpy.shutdown()
        return

    node = AddTwoIntsClient()
    a = int(sys.argv[1])
    b = int(sys.argv[2])
    response = node.send_request(a, b)
    if response:
        node.get_logger().info(f'ایڈ ٹو انٹس کا نتیجہ: {a} + {b} = {response.sum}')
    else:
        node.get_logger().error('سروس کال ناکام ہو گئی۔')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
سروس مثالوں کو چلانے کے لیے:
1.  یقینی بنائیں کہ آپ کے پاس ایک کسٹم ROS 2 پیکیج ہے جہاں `AddTwoInts.srv` کی وضاحت اور تعمیر کی گئی ہے۔
2.  سرور لانچ کریں: `ros2 run <package_name> add_two_ints_server`
3.  کلائنٹ لانچ کریں: `ros2 run <package_name> add_two_ints_client 5 7`

## عملی مثال: فرضی موٹر کمانڈز بھیجنا

آئیے ایک مکمل مثال بنائیں جہاں ایک پائتھن AI ایجنٹ (یہاں نقلی) وقتاً فوقتاً `cmd_vel` ٹاپک پر `Twist` پیغام شائع کرتا ہے، جو روبوٹ کے آگے بڑھنے کی نقالی کرتا ہے۔

سب سے پہلے، یقینی بنائیں کہ آپ کے پاس سمولیشن میں ایک روبوٹ ہے (جیسے گیزیبو یا آئزک سم) جو `/cmd_vel` کو سبسکرائب کرتا ہے اور انہیں حقیقی روبوٹ کی حرکت میں ترجمہ کرتا ہے۔

**`simple_ai_agent.py`:**

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SimpleAIAgent(Node):
    def __init__(self):
        super().__init__('simple_ai_agent')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_period = 0.5 # ہر 0.5 سیکنڈ میں کمانڈز شائع کریں
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('سادہ AI ایجنٹ شروع ہو گیا۔')
        self.forward_speed = 0.1 # m/s
        self.turn_speed = 0.0 # rad/s

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.forward_speed
        msg.angular.z = self.turn_speed
        self.publisher_.publish(msg)
        self.get_logger().info(f'کمانڈنگ: لکیری X={self.forward_speed:.2f}, زاویائی Z={self.turn_speed:.2f}')

def main(args=None):
    rclpy.init(args=args)
    agent_node = SimpleAIAgent()
    rclpy.spin(agent_node)
    agent_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
اسے چلانے کے لیے:
1.  اپنا نقلی روبوٹ لانچ کریں (جیسے گیزیبو میں)۔
2.  `python simple_ai_agent.py` چلائیں۔
آپ کا روبوٹ سمولیشن میں آگے بڑھنا شروع کر دینا چاہیے۔ پھر آپ کوڈ میں `forward_speed` یا `turn_speed` میں ترمیم کر کے مختلف حرکات کے ساتھ تجربہ کر سکتے ہیں۔

یہ عملی مثال ظاہر کرتی ہے کہ کس طرح سادہ پائتھن `rclpy` نوڈس ROS 2 ٹاپکس کے ذریعے ہیومنائڈ روبوٹ کی حرکات کو براہ راست کنٹرول کر سکتے ہیں۔