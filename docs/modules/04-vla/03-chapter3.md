# Chapter 3: Advanced Task Planning with Large Language Models

## 3.1 From Simple Commands to Complex Plans

In the previous chapter, we built a system that could respond to simple, direct commands. This is a great start, but the true frontier of human-robot interaction lies in giving robots the ability to understand complex, high-level instructions and formulate a plan to achieve them. This is where Large Language Models (LLMs) transform robotics.

A simple keyword-based system can understand "go forward," but it has no hope of understanding, "Go to the kitchen, find my keys, and let me know if you see them." An LLM, with its vast world knowledge and reasoning capabilities, can break this down:
1.  **Goal:** Find keys for the user.
2.  **Constraint:** The keys are likely in the kitchen.
3.  **Action 1:** Navigate to the "kitchen" location.
4.  **Action 2:** Visually search for an object that looks like "keys".
5.  **Action 3 (Conditional):** If keys are found, report success to the user.

This process of taking a high-level goal and decomposing it into a sequence of executable, primitive actions is the essence of LLM-powered task planning.

***
*Placeholder for a detailed diagram: "The LLM Task Planning Loop."
1.  **User Input:** A speech bubble with a complex command: "Bring me the soda from the fridge."
2.  **LLM Planner Node:** A box that receives the command. Inside, it shows a "Prompt" being formulated with the command and a list of available robot actions.
3.  **Cloud API:** The prompt is sent to a cloud icon labeled "OpenAI GPT-4 API".
4.  **Generated Plan:** The API returns a structured plan (e.g., a JSON or numbered list) with steps like `navigate_to("kitchen")`, `open("fridge")`, `find_object("soda")`, `grasp("soda")`, `close("fridge")`, `navigate_to("user")`.
5.  **Plan Executor Node:** A box that receives this plan and iterates through it.
6.  **ROS 2 Actions/Services:** The Plan Executor sends goals to various ROS 2 Action Servers, like `nav2_client`, `moveit_client`, etc. These servers control the robot's hardware.
7.  **Feedback Loop:** An arrow goes from the execution back to the LLM Planner, labeled "Feedback (Success/Failure)". This shows that if a step fails, the planner can be reinvoked to create a new plan.
***

## 3.2 Designing the Robot's "API": The Action Library

An LLM cannot directly control a robot's motors. We must provide it with a "robot API"—a well-defined set of primitive actions, or functions, that it can call. The design of this action library is one of the most critical aspects of building an LLM-powered robot.

**Principles of Action Design:**

*   **Abstraction:** Actions should be at a level of abstraction that makes sense for high-level planning. Instead of `set_wheel_velocity(v_left, v_right)`, you should have `navigate_to(location)`.
*   **Success/Failure Conditions:** Each action must have clear conditions for success and failure that can be reported back to the planner. The `navigate_to` action succeeds if the robot reaches the goal coordinates and fails if it gets stuck.
*   **Grounded in Reality:** Each action must correspond to a robust, underlying robotics capability. Don't create a `find_keys` action if you don't have a reliable object detection model to support it.

**Example Action Library for a Mobile Manipulator:**

```python
# This is a conceptual Python representation of our robot's capabilities.
# We will expose these function signatures to the LLM in our prompt.

class RobotActions:
    def navigate_to(self, location: str) -> bool:
        """Moves the robot to a predefined, named location in the environment.
        Locations can be: 'kitchen', 'living_room', 'charging_dock'.
        Returns True on success, False on failure."""
        # ... implementation using ROS 2 Nav2 action client ...
        pass

    def find_object(self, object_description: str, search_area: str) -> str:
        """
        Visually searches for an object matching the description within a search area.
        Returns the unique name of the found object (e.g., 'soda_can_1') or 'None' if not found.
        """
        # ... implementation using a perception pipeline (e.g., YOLO, DETIC) ...
        pass

    def pick_up(self, object_name: str) -> bool:
        """Grasps the specified object. The object must have been found first."""
        # ... implementation using ROS 2 MoveIt 2 action client ...
        pass
    
    def say(self, text_to_speak: str) -> bool:
        """Speaks the given text out loud."""
        # ... implementation using a text-to-speech engine ...
        pass
```
*This documented set of functions is exactly what we will insert into our prompt to the LLM.*

## 3.3 The Core of the System: The LLM Planner Node

This ROS 2 node orchestrates the entire process. It takes the user's transcribed voice command, wraps it in a carefully engineered prompt, sends it to the GPT API, and then parses the response to extract a machine-readable plan.

**Prompt Engineering is Key:**

The "art" of getting good results from an LLM is prompt engineering. A good prompt for task planning should include:

1.  **The Role:** "You are a helpful robot assistant..."
2.  **The Goal:** "...your task is to decompose a user's command into a plan."
3.  **The Tools:** The full definition of your action library, including function signatures and docstrings.
4.  **Output Format:** Instructions on how the output should be formatted (e.g., JSON, a numbered list).
5.  **One-Shot or Few-Shot Examples:** One or more examples of a user command and the corresponding correct plan. This helps the LLM understand the desired output format and style.

**The `llm_planner_node.py` with Advanced Prompting:**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import json

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner_node')
        self.subscription = self.create_subscription(
            String, 'transcribed_text', self.command_callback, 10)
        self.plan_publisher = self.create_publisher(String, 'plan', 10)
        
        # Load the action library description from a file
        with open("action_library.txt", "r") as f:
            self.action_library = f.read()

        # Load the few-shot examples
        with open("plan_examples.txt", "r") as f:
            self.plan_examples = f.read()

    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received command: "{command}"')

        prompt = f"""
You are an expert robot control system. Your task is to take a user's command and convert it into a structured, machine-readable plan in JSON format.
You can only use the functions defined in the Action Library. Do not make up new functions.

<Action_Library>
{self.action_library}
</Action_Library>

Here are some examples:
<Examples>
{self.plan_examples}
</Examples>

User command: "{command}"

Generate the JSON plan.
"""
        try:
            response = openai.chat.completions.create(
                model="gpt-4",
                messages=[
                    {"role": "system", "content": prompt}
                ],
                response_format={"type": "json_object"}
            )
            plan_json = response.choices[0].message.content
            self.get_logger().info(f'Generated plan:\n{plan_json}')

            # Validate the JSON plan here before publishing
            # ...

            plan_msg = String()
            plan_msg.data = plan_json
            self.plan_publisher.publish(plan_msg)

        except (openai.APIError, json.JSONDecodeError) as e:
            self.get_logger().error(f"Error generating or parsing plan: {e}")

# main function as before...
```
*This improved node uses a more structured prompt, loading the action library and examples from external files for better organization. It also requests a JSON object as output for more reliable parsing.*

## 3.4 The Hands: A ROS 2 Plan Executor

A plan is just text until something executes it. The Plan Executor node is the "doer." It subscribes to the plan topic and is responsible for invoking the real ROS 2 services and actions that correspond to the actions in the plan.

**Conceptual Logic for a Plan Executor:**

This node would be implemented as a state machine or a simple loop.

1.  Subscribe to the `/plan` topic.
2.  On receiving a JSON plan, parse it into a queue of steps.
3.  For each step in the queue:
    *   **Get Action:** Get the action name (e.g., `navigate_to`).
    *   **Get Arguments:** Get the arguments (e.g., `location: "kitchen"`).
    *   **Dispatch:** Look up the action name in a dictionary that maps action names to Python functions that can call the real ROS 2 clients.
    *   **Execute and Wait:** Call the corresponding function (e.g., `self.ros_nav2_client.go_to_pose(...)`). This function would be asynchronous and would block until the action server returns a result (success, failure, or preempted).
    *   **Handle Feedback:**
        *   If the action succeeds, log it and proceed to the next step.
        *   If the action fails, the executor could stop and report failure, or it could even send a message back to the LLM planner with the error ("Failed to navigate to kitchen: path was blocked"), allowing the LLM to generate a new, corrective plan.

*Developing a full-featured, asynchronous plan executor with feedback is a significant engineering task involving ROS 2 action clients, but this structure provides a clear roadmap.*

## 3.5 Summary and Next Steps

In this culminating chapter, you have learned the principles of modern, LLM-driven robotics. You saw how to move beyond simple commands to a system of true task planning by:
1.  Designing a well-defined **action library**.
2.  Using sophisticated **prompt engineering** to have an LLM generate plans.
3.  Architecting a **plan executor** to turn those plans into reality.

This fusion of the LLM's reasoning and the robot's physical capabilities is what enables truly intelligent and flexible behavior. From here, you are equipped with the foundational knowledge to explore even more advanced topics like VLM-based perception (asking an LLM "what do you see?"), dynamic replanning, and learning new skills from observation. Congratulations on completing this module on Vision-Language Models!
