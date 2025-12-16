# باب 3: URDF میں روبوٹ کے جسم کی تعمیر

## URDF کیا ہے اور ہیومنائڈ روبوٹس کو اس کی ضرورت کیوں ہے

**یونیفائیڈ روبوٹ ڈسکرپشن فارمیٹ (URDF)** ROS میں استعمال ہونے والا ایک XML فائل فارمیٹ ہے جو روبوٹ کے تمام عناصر کو بیان کرتا ہے۔ یہ کسی بھی روبوٹ ڈویلپر کے لیے ایک بنیادی ٹول ہے، خاص طور پر ہیومنائڈز جیسے پیچیدہ نظاموں کے لیے۔ ایک URDF فائل روبوٹ کی مکمل وضاحت کرتی ہے:

*   **کائیمیٹک اور ڈائنامک خصوصیات**: روبوٹ کی ساخت کو بیان کرتا ہے، بشمول اس کے لنکس (سخت جسم) اور جوڑ (لنکس کے درمیان رابطے)۔ یہ ہر لنک کے لیے انرشیل خصوصیات (کمیت، انرشیا میٹرکس) کی بھی وضاحت کرتا ہے۔
*   **بصری خصوصیات**: ہر لنک کی بصری ظاہری شکل کی وضاحت کرتا ہے، بشمول اس کا 3D میش ماڈل، رنگ، اور بناوٹ۔ یہ سمیلیٹرز (جیسے گیزیبو) یا ویژولائزیشن ٹولز (جیسے RViz) میں روبوٹ کو رینڈر کرنے کے لیے استعمال ہوتا ہے۔
*   **ٹکراؤ کی خصوصیات**: سمیلیشنز میں ٹکراؤ کا پتہ لگانے کے لیے استعمال ہونے والی سادہ ہندسی اشکال کی وضاحت کرتا ہے۔ یہ اکثر بصری میشز سے زیادہ سادہ ہوتے ہیں تاکہ ٹکراؤ کے حساب کو تیز کیا جا سکے۔

ہیومنائڈ روبوٹس کے لیے، URDF ناگزیر ہے کیونکہ:

*   **پیچیدگی**: ہیومنائڈز میں بہت سے لنکس اور جوڑ ہوتے ہیں (ٹانگیں، بازو، دھڑ، سر)، جس سے ایک تفصیلی، منظم تفصیل ضروری ہو جاتی ہے۔
*   **سمیلیشن**: URDF فائلیں فزکس سمیلیٹرز (جیسے گیزیبو) کے ذریعہ روبوٹ کے جسمانی رویے کو ماڈل کرنے کے لیے براہ راست استعمال ہوتی ہیں۔
*   **ویژولائزیشن**: RViz جیسے ٹولز URDF کا استعمال روبوٹ کی ساخت اور اس کی موجودہ حالت (جیسے جوڑوں کے زاویے) کو ظاہر کرنے کے لیے کرتے ہیں۔
*   **کائیمیٹکس/ڈائنامکس لائبریریاں**: انورس کائیمیٹکس یا ڈائنامکس کے حساب کے لیے سافٹ ویئر پیکجز روبوٹ کی ساخت کو سمجھنے کے لیے URDF کو پارس کرتے ہیں۔

## لنکس، جوڑ، اور کائیمیٹک چینز

ایک URDF ماڈل دو بنیادی عناصر سے بنتا ہے: `<link>` اور `<joint>`۔

### `<link>` عنصر: سخت اجسام

ایک `<link>` عنصر روبوٹ کے ایک سخت جسم والے حصے کو بیان کرتا ہے۔ یہ روبوٹ کا دھڑ، اوپری بازو، ران، یا پاؤں ہو سکتا ہے۔ ہر لنک میں ہوتا ہے:

*   **انرشیل خصوصیات**: `<inertial>` ٹیگ کے ذریعہ بیان کیا گیا ہے، جس میں `mass` اور `inertia` میٹرکس شامل ہیں۔ یہ درست فزکس سمیلیشن کے لیے اہم ہیں۔
*   **بصری خصوصیات**: `<visual>` ٹیگ کے ذریعہ بیان کیا گیا ہے، جو لنک کی جیومیٹری (میش، باکس، سلنڈر، کرہ) اور مواد (رنگ، بناوٹ) کی وضاحت کرتا ہے۔
*   **ٹکراؤ کی خصوصیات**: `<collision>` ٹیگ کے ذریعہ بیان کیا گیا ہے، جو ٹکراؤ کا پتہ لگانے کے لیے استعمال ہونے والی جیومیٹری کی وضاحت کرتا ہے۔ یہ اکثر بصری کی ایک سادہ شکل ہوتی ہے۔

### `<joint>` عنصر: لنکس کے درمیان رابطے

ایک `<joint>` عنصر دو لنکس کے درمیان رابطے کی کائیمیٹک اور ڈائنامک خصوصیات کو بیان کرتا ہے۔ جوڑ یہ بتاتے ہیں کہ لنکس ایک دوسرے کے نسبت کیسے حرکت کر سکتے ہیں۔ کلیدی صفات میں شامل ہیں:

*   **`name`**: جوڑ کے لیے ایک منفرد شناخت کنندہ۔
*   **`type`**: جوڑ کی حرکت کی صلاحیتوں کی وضاحت کرتا ہے (جیسے `revolute` ایک گھومنے والے جوڑ کے لیے جیسے قبضہ، `prismatic` ایک سلائڈنگ جوڑ کے لیے، `fixed` ایک سخت کنکشن کے لیے)۔
*   **`parent`**: روبوٹ کی بنیاد یا اصلیت کے قریب والے لنک کا نام۔
*   **`child`**: روبوٹ کی بنیاد سے دور والے لنک کا نام۔
*   **`origin`**: پیرنٹ لنک کے نسبت جوڑ کی پوزیشن اور سمت کی وضاحت کرتا ہے۔
*   **`axis`**: ریولوٹ اور پرزمیٹک جوڑوں کے لیے، گردش یا ترجمہ کا محور بتاتا ہے۔
*   **`limit`**: جوڑ کی حرکت کی حد (نچلی/اوپری حدود)، رفتار، اور کوشش کی حدود کی وضاحت کرتا ہے۔

### کائیمیٹک چینز

جوڑوں کے ذریعے جڑے ہوئے لنکس کی ایک سیریز ایک **کائیمیٹک چین** بناتی ہے۔ ہیومنائڈز کے لیے، آپ کے پاس متعدد چینز ہوں گی: ہر ٹانگ کے لیے ایک، ہر بازو کے لیے ایک، دھڑ/سر کے لیے ایک، سب ایک مرکزی بیس لنک (جیسے دھڑ یا کولہے) سے شاخیں نکالتی ہیں۔

## ایک بنیادی ہیومنائڈ URDF ماڈل بنانا

آئیے ایک دو ٹانگوں والے روبوٹ کے لیے ایک بہت ہی سادہ URDF ماڈل بنائیں۔ یہ لنکس اور جوڑوں کی بنیادی ساخت کا مظاہرہ کرے گا۔ یہ ماڈل `Humanoid_Robotics/simulations/module-1/simple_humanoid.urdf` کے طور پر محفوظ کیا جائے گا۔

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- بیس لنک: دھڑ -->
  <link name="torso">
    <visual>
      <geometry><box size="0.2 0.1 0.3"/></geometry>
      <material name="blue"><color rgba="0 0 1 1"/></material>
    </visual>
    <collision>
      <geometry><box size="0.2 0.1 0.3"/></geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- بائیں ٹانگ -->
  <link name="left_thigh">
    <visual>
      <geometry><box size="0.05 0.05 0.2"/></geometry>
      <material name="green"><color rgba="0 1 0 1"/></material>
    </visual>
    <collision>
      <geometry><box size="0.05 0.05 0.2"/></geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="torso_to_left_thigh" type="revolute">
    <parent link="torso"/>
    <child link="left_thigh"/>
    <origin xyz="0 0.075 -0.15" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
  </joint>

  <link name="left_shin">
    <visual>
      <geometry><box size="0.05 0.05 0.2"/></geometry>
      <material name="green"/>
    </visual>
    <collision>
      <geometry><box size="0.05 0.05 0.2"/></geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="left_thigh_to_left_shin" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shin"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="0" effort="100" velocity="10"/>
  </joint>

  <link name="left_foot">
    <visual>
      <geometry><box size="0.07 0.05 0.03"/></geometry>
      <material name="green"/>
    </visual>
    <collision>
      <geometry><box size="0.07 0.05 0.03"/></geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>
  <joint name="left_shin_to_left_foot" type="revolute">
    <parent link="left_shin"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.785" upper="0.785" effort="100" velocity="10"/>
  </joint>

  <!-- دائیں ٹانگ (اسی طرح کی ساخت، صرف Y میں آئینہ دار) -->
  <link name="right_thigh">
    <visual>
      <geometry><box size="0.05 0.05 0.2"/></geometry>
      <material name="green"/>
    </visual>
    <collision>
      <geometry><box size="0.05 0.05 0.2"/></geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="torso_to_right_thigh" type="revolute">
    <parent link="torso"/>
    <child link="right_thigh"/>
    <origin xyz="0 -0.075 -0.15" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
  </joint>

  <link name="right_shin">
    <visual>
      <geometry><box size="0.05 0.05 0.2"/></geometry>
      <material name="green"/>
    </visual>
    <collision>
      <geometry><box size="0.05 0.05 0.2"/></geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="right_thigh_to_right_shin" type="revolute">
    <parent link="right_thigh"/>
    <child link="right_shin"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="0" effort="100" velocity="10"/>
  </joint>

  <link name="right_foot">
    <visual>
      <geometry><box size="0.07 0.05 0.03"/></geometry>
      <material name="green"/>
    </visual>
    <collision>
      <geometry><box size="0.07 0.5 0.03"/></geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>
  <joint name="right_shin_to_right_foot" type="revolute">
    <parent link="right_shin"/>
    <child link="right_foot"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.785" upper="0.785" effort="100" velocity="10"/>
  </joint>

  <material name="blue">
    <color rgba="0 0 1 1"/>
  </material>
  <material name="green">
    <color rgba="0 1 0 1"/>
  </material>

</robot>