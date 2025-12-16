# باب 3: روبوٹ سینسرز کی نقالی

## سینسر سمیلیشن کا تعارف

سینسرز روبوٹ کی آنکھیں، کان اور لمس ہیں۔ وہ روبوٹ کی اندرونی حالت اور اس کے ارد گرد کے ماحول کے بارے میں اہم معلومات فراہم کرتے ہیں، جس سے پرسیپشن، نیویگیشن اور تعامل ممکن ہوتا ہے۔ تاہم، حقیقی دنیا کے سینسرز مہنگے، نازک ہو سکتے ہیں اور انہیں محتاط انشانکن کی ضرورت ہوتی ہے۔ یہ وہ جگہ ہے جہاں **سینسر سمیلیشن** انمول ہو جاتی ہے۔

نقلی سینسرز ہمیں اجازت دیتے ہیں:

*   **الگورتھم تیار اور ٹیسٹ کریں**: پرسیپشن الگورتھم (جیسے آبجیکٹ ڈیٹیکشن، SLAM کے لیے) کو بغیر فزیکل ہارڈویئر کے لکھیں اور ڈی بگ کریں۔
*   **مصنوعی ڈیٹا تیار کریں**: مشین لرننگ ماڈلز کی تربیت کے لیے بڑے، متنوع ڈیٹاسیٹ بنائیں، حقیقی دنیا کے ڈیٹا اکٹھا کرنے کی حدود پر قابو پاتے ہوئے۔
*   **کنفیگریشنز کے ساتھ تجربہ کریں**: بغیر جسمانی ترمیم کے مختلف سینسر پلیسمنٹ، اقسام اور پیرامیٹرز کو تیزی سے ٹیسٹ کریں۔
*   **سینسر کی حدود کو سمجھیں**: تجزیہ کریں کہ شور، رکاوٹیں اور ماحولیاتی عوامل سینسر ریڈنگز پر کیسے اثر انداز ہوتے ہیں۔

یہ باب آپ کو گیزیبو اور یونٹی دونوں میں عام روبوٹ سینسرز کو ترتیب دینے میں رہنمائی کرے گا، جس میں LiDAR، ڈیپتھ کیمرے، اور IMUs پر توجہ دی جائے گی۔

## گیزیبو میں LiDAR سینسر شامل کرنا

**LiDAR (لائٹ ڈیٹیکشن اینڈ رینجنگ)** سینسرز اشیاء سے فاصلے کی پیمائش کرتے ہیں جو کہ دھڑکن والی لیزر لائٹ خارج کرتے ہیں اور منعکس شدہ روشنی کو واپس آنے میں لگنے والے وقت کی پیمائش کرتے ہیں۔ وہ 3D میپنگ، رکاوٹوں سے بچنے اور نیویگیشن کے لیے اہم ہیں۔

گیزیبو میں، آپ ایک پلگ ان کا استعمال کرتے ہوئے ایک نقلی LiDAR شامل کر سکتے ہیں۔ اس میں عام طور پر آپ کے روبوٹ کی SDF یا URDF فائل میں ترمیم کرنا شامل ہوتا ہے۔

```xml
<link name="lidar_link">
  <visual>
    <geometry><box><size>0.05 0.05 0.05</size></box></geometry>
  </visual>
  <collision>
    <geometry><box><size>0.05 0.05 0.05</size></box></geometry>
  </collision>
</link>

<joint name="lidar_joint" type="fixed">
  <parent>base_link</parent> <!-- اپنے روبوٹ کے بیس لنک سے منسلک کریں -->
  <child>lidar_link</child>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>

<gazebo reference="lidar_link">
  <sensor name="lidar_sensor" type="ray">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-1.570796</min_angle>
          <max_angle>1.570796</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="gazebo_ros_lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <argument>~/out:=scan</argument>
        <namespace>/my_robot</namespace>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```
*خود جانچ*: اپنے روبوٹ ماڈل میں یہ شامل کرنے اور گیزیبو میں روبوٹ لانچ کرنے کے بعد، آپ کو ایک نیا ROS 2 ٹاپک `/my_robot/scan` نظر آنا چاہیے جو `sensor_msgs/LaserScan` پیغامات شائع کر رہا ہو۔

## گیزیبو میں ڈیپتھ کیمرا پلگ ان شامل کرنا

**ڈیپتھ کیمرے** RGB (رنگ) تصاویر اور ایک ڈیپتھ میپ (اشیاء سے فاصلہ) دونوں فراہم کرتے ہیں۔ یہ امتزاج 3D تعمیر نو، آبجیکٹ کی شناخت، اور پکڑنے کے کاموں کے لیے انمول ہے۔

گیزیبو پلگ ان کے ذریعے ڈیپتھ کیمروں کی حمایت کرتا ہے۔ یہاں ایک عام کنفیگریشن سنیپٹ ہے:

```xml
<link name="depth_camera_link">
  <visual>
    <geometry><box><size>0.03 0.03 0.03</size></box></geometry>
  </visual>
  <collision>
    <geometry><box><size>0.03 0.03 0.03</size></box></geometry>
  </collision>
</link>

<joint name="depth_camera_joint" type="fixed">
  <parent>base_link</parent>
  <child>depth_camera_link</child>
  <origin xyz="0 0 0.15" rpy="0 0 0"/>
</joint>

<gazebo reference="depth_camera_link">
  <sensor name="depth_camera_sensor" type="depth">
    <always_on>1</always_on>
    <update_rate>30.0</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.05</near>
        <far>3.0</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_depth_camera.so">
      <ros>
        <argument>image_raw:=depth/image_raw</argument>
        <argument>camera_info:=depth/camera_info</argument>
        <argument>depth/image_raw:=depth/depth_image_raw</argument>
        <namespace>/my_robot/camera</namespace>
      </ros>
      <frame_name>depth_camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```
*خود جانچ*: لانچ ہونے پر، آپ کو ROS 2 ٹاپکس جیسے `/my_robot/camera/depth/image_raw` اور `/my_robot/camera/depth/depth_image_raw` نظر آنے چاہئیں جو تصویری پیغامات شائع کر رہے ہوں۔

## گیزیبو میں IMU سینسر پلگ ان شامل کرنا

ایک **IMU (انرشیل میژرمنٹ یونٹ)** روبوٹ کی مخصوص قوت (ایکسلریشن) اور زاویائی رفتار کی پیمائش کرتا ہے۔ یہ ڈیٹا روبوٹ کی سمت، رفتار اور پوزیشن کا تخمینہ لگانے کے لیے اہم ہے، خاص طور پر نیویگیشن اور کنٹرول کے کاموں میں۔

گیزیبو IMU پلگ ان ایکسلرومیٹر اور جائروسکوپ کی نقالی کرتے ہیں:

```xml
<link name="imu_link">
  <visual>
    <geometry><box><size>0.01 0.01 0.01</size></box></geometry>
  </visual>
  <collision>
    <geometry><box><size>0.01 0.01 0.01</size></box></geometry>
  </collision>
</link>

<joint name="imu_joint" type="fixed">
  <parent>base_link</parent>
  <child>imu_link</child>
  <origin xyz="0 0 0.16" rpy="0 0 0"/>
</joint>

<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>1</always_on>
    <update_rate>100</update_rate>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/my_robot/imu</namespace>
        <argument>~/out:=imu_data</argument>
      </ros>
      <frame_name>imu_link</frame_name>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>
  </sensor>
</gazebo>
```
*خود جانچ*: ٹاپک `/my_robot/imu/imu_data` کو `sensor_msgs/Imu` پیغامات `linear_acceleration` اور `angular_velocity` ڈیٹا کے ساتھ شائع کرنا چاہیے۔

## یونٹی پرسیپشن پیکیج کے ساتھ کیمرا سینسر شامل کرنا

**یونٹی پرسیپشن پیکیج** کمپیوٹر ویژن کے کاموں کے لیے مصنوعی ڈیٹاسیٹ بنانے کے لیے ڈیزائن کیا گیا ہے۔ یہ آپ کو اپنے یونٹی منظر میں مختلف قسم کے کیمرا سینسرز (RGB، ڈیپتھ، سیمنٹک سیگمنٹیشن، وغیرہ) شامل کرنے اور انہیں مشین لرننگ کے لیے موزوں ایک منظم فارمیٹ میں ڈیٹا آؤٹ پٹ کرنے کے لیے ترتیب دینے کی اجازت دیتا ہے۔

1.  **پرسیپشن پیکیج انسٹال کریں**:
    *   اپنے یونٹی پروجیکٹ میں، `Window > Package Manager` پر جائیں۔
    *   `+` آئیکن پر کلک کریں اور "Add package from git URL..." کا انتخاب کریں۔
    *   `com.unity.perception` درج کریں اور "Add" پر کلک کریں۔

2.  **اپنے منظر میں کیمرا شامل کریں**:
    *   ہیرارکی ونڈو میں دائیں کلک کریں۔
    *   `3D Object > Camera`۔ روبوٹ کے منظر کی نقالی کرنے کے لیے کیمرے کی پوزیشن بنائیں۔
    *   کیمرے کو منتخب کریں، پھر انسپکٹر میں، "Add Component" پر کلک کریں اور "Perception Camera" تلاش کریں۔ اسے شامل کریں۔

3.  **پرسیپشن کیمرا ترتیب دیں**:
    *   **کیپچر وقفہ**: کیمرا کتنی بار ڈیٹا کیپچر کرتا ہے۔
    *   **آؤٹ پٹ ڈائرکٹری**: کیپچر کیا گیا ڈیٹا کہاں محفوظ کیا جائے گا۔
    *   **لیبلرز**: یہ بتانے کے لیے لیبلرز شامل کریں کہ کس قسم کا ڈیٹا تیار کرنا ہے (جیسے `RGB Camera Labeler`، `Depth Camera Labeler`)۔
    *   **کیمرا پیرامیٹرز**: FOV، کلپ پلینز وغیرہ کو ایڈجسٹ کریں۔

جب آپ اپنا یونٹی منظر چلاتے ہیں، تو پرسیپشن کیمرا مخصوص سینسر ڈیٹا (جیسے RGB تصاویر) کو آپ کی آؤٹ پٹ ڈائرکٹری میں محفوظ کرے گا، جو AI تربیت میں استعمال کے لیے تیار ہے۔

## RViz2 کے ساتھ گیزیبو سینسر ڈیٹا کو ویژولائز کرنا

**RViz2** ROS 2 کے لیے ایک 3D ویژولائزیشن ٹول ہے۔ یہ آپ کو مختلف قسم کے روبوٹ ڈیٹا، بشمول گیزیبو سے سینسر ریڈنگز، کو گرافیکل ماحول میں ظاہر کرنے کی اجازت دیتا ہے۔

1.  **RViz2 لانچ کریں**:
    ```bash
    rviz2
    ```

2.  **ڈسپلے شامل کریں**:
    *   RViz2 ونڈو میں، "ڈسپلے" پینل (نیچے بائیں) میں "Add" بٹن پر کلک کریں۔
    *   **LiDAR کے لیے**: `LaserScan` منتخب کریں اور ٹاپک کو `/my_robot/scan` پر سیٹ کریں۔
    *   **ڈیپتھ کیمرے کے لیے**: `Image` منتخب کریں اور ٹاپک کو `/my_robot/camera/depth/image_raw` یا `/my_robot/camera/depth/depth_image_raw` پر سیٹ کریں۔
    *   **IMU کے لیے**: `IMU` منتخب کریں اور ٹاپک کو `/my_robot/imu/imu_data` پر سیٹ کریں۔
    *   **فکسڈ فریم سیٹ کریں**: "گلوبل آپشنز" سیکشن میں، "فکسڈ فریم" کو `map` یا اپنے روبوٹ کے بیس فریم (جیسے `base_link`) پر سیٹ کریں۔

RViz2 میں سینسر ڈیٹا کو ویژولائز کرکے، آپ تصدیق کر سکتے ہیں کہ آپ کے گیزیبو سینسرز صحیح طریقے سے ترتیب دیے گئے ہیں اور متوقع معلومات شائع کر رہے ہیں، جو آپ کے سمیلیشن سیٹ اپ کے لیے ایک اہم فیڈ بیک لوپ فراہم کرتا ہے۔

یہ ڈیجیٹل ٹوئنز بنانے اور سینسرز کی نقالی پر ماڈیول 2 کا اختتام ہے۔