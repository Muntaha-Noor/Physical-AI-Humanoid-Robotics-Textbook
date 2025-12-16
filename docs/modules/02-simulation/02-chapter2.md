# Chapter 2: Human-Robot Interaction in Unity

## Unity's Role in High-Fidelity Rendering and HRI

While Gazebo excels at physics-based simulation and integration with ROS 2, **Unity** stands out for its high-fidelity rendering capabilities and robust tools for creating interactive 3D environments. For humanoid robotics, Unity becomes invaluable when the focus shifts to:

*   **Human-Robot Interaction (HRI)**: Creating realistic scenarios where humans and robots interact, allowing for testing of intuitive interfaces, safety protocols, and collaborative tasks. Unity's rich visual tools and animation systems are perfect for this.
*   **Photorealistic Environments**: Generating visually stunning environments that mimic the real world, crucial for training vision-based AI algorithms where the visual details (lighting, shadows, textures) significantly impact model performance.
*   **User Interface Development**: Leveraging Unity's UI toolkit to build sophisticated dashboards and control panels for interacting with simulated robots.

By combining Unity's strengths with Gazebo's physics and ROS 2 integration, we create a powerful digital twin ecosystem capable of addressing a wider range of research and development challenges in humanoid robotics.

## Setting Up a New Unity Project with HDRP

The **High Definition Render Pipeline (HDRP)** is a pre-built Scriptable Render Pipeline in Unity that allows you to create cutting-edge, high-fidelity graphics. It's ideal for projects that demand photorealism, such as humanoid robot simulations for HRI or realistic synthetic data generation.

Here's a guide to setting up a new Unity project with HDRP:

1.  **Launch Unity Hub**: Open Unity Hub and click "New Project".
2.  **Select HDRP Template**: From the templates, choose "3D (HDRP)". Give your project a name (e.g., `HumanoidHRI`) and select a location. Click "Create Project".
3.  **Project Initialization**: Unity will take some time to create the project and import all necessary HDRP assets.
4.  **Verify HDRP Setup**: Once the project opens, go to `Edit > Project Settings > Graphics`. Ensure "HDRP Global Settings" is assigned. Also, check `Edit > Project Settings > Quality`. You should see quality levels configured for HDRP.
5.  **Scene Setup**: The default HDRP scene often includes a basic setup with lighting. You can delete existing objects if you want to start from scratch, but keep the `Volume` GameObject which controls global post-processing effects.

Your Unity project is now configured for high-fidelity rendering.

## Importing a Humanoid URDF Model

To bring your humanoid robot designs (created in URDF, as discussed in Module 1) into Unity, you'll use the **Unity Robotics URDF Importer** package. This package can parse URDF files and convert them into Unity GameObjects, complete with colliders, rigidbodies, and configurable joints.

1.  **Install URDF Importer Package**:
    *   In your Unity project, go to `Window > Package Manager`.
    *   Click the `+` icon in the top-left corner and choose "Add package from git URL...".
    *   Enter `com.unity.robotics.urdf-importer` and click "Add".

2.  **Import Your URDF**:
    *   In the Project window, right-click and choose `Create > Robotics > URDF Robot`.
    *   Locate your humanoid URDF file (e.g., `simulations/module-1/robot.urdf`) and select it.
    *   The importer will create a prefab of your robot in the Project window. Drag this prefab into your scene hierarchy.
    *   You may need to adjust the robot's position and rotation to place it correctly in your scene.

Your humanoid robot model is now part of your Unity scene, ready for interaction and rendering.

## Applying Realistic Lighting with HDRP

Realistic lighting is crucial for creating photorealistic environments and accurate vision-based sensor simulation. HDRP offers a rich set of tools to achieve this.

### 1. Global Illumination

Global Illumination (GI) simulates how light bounces off surfaces, creating more realistic indirect lighting.

*   **Baked GI**: For static scenes, pre-calculate light bounces for performance. Go to `Window > Rendering > Lighting`, then `Generate Lighting`.
*   **Realtime GI**: For dynamic scenes, HDRP supports real-time GI with limitations.

### 2. Light Types

*   **Directional Light**: Simulates light from a distant source (like the sun). Adjust its rotation to change the time of day and shadow direction.
*   **Point Light**: Emits light in all directions from a single point (e.g., a bare lightbulb).
*   **Spot Light**: Emits light in a cone shape (e.g., a flashlight).
*   **Area Light**: Emits light from a 2D surface, useful for soft, diffused lighting.

### 3. Exposure Control

HDRP uses physical lighting units, so **Exposure** is critical to balance the scene's brightness. You can control this via the `Exposure` volume override, either automatically or manually.

### 4. Post-Processing Effects

HDRP comes with a suite of post-processing effects that can dramatically enhance realism:

*   **Bloom**: Creates a halo around bright areas.
*   **Vignette**: Darkens the edges of the image.
*   **Color Grading**: Adjusts the overall color balance, contrast, and saturation.
*   **Screen Space Reflections (SSR)**: Simulates reflections on glossy surfaces.
*   **Ambient Occlusion (SSAO)**: Adds subtle shadows where objects are close together.

To add these, select your `Global Volume` in the Hierarchy, then in the Inspector, click "Add Override" and choose the desired effect.

## Creating and Applying Physically Based Rendering (PBR) Materials

**Physically Based Rendering (PBR)** materials simulate how light interacts with surfaces in a way that closely matches real-world physics. This makes objects look much more realistic, which is vital for photorealistic rendering in HRI and synthetic data generation.

PBR materials typically use several textures to define a surface's properties:

1.  **Albedo Map (Base Color)**: Defines the base color of the surface.
2.  **Normal Map**: Adds surface detail by faking bumps and dents without adding actual geometry.
3.  **Metallic Map**: Defines which parts of the surface are metallic and which are dielectric (non-metallic).
4.  **Smoothness Map (Roughness/Glossiness)**: Defines how rough or smooth the surface is, affecting how light scatters or reflects.
5.  **Ambient Occlusion Map (AO)**: Simulates soft shadows where objects are close together, adding depth.

### Creating a PBR Material

1.  **In the Project window**, navigate to a suitable folder (e.g., `Assets/Materials`).
2.  **Right-click > Create > Material`.
3.  **In the Inspector window**, change the `Shader` dropdown to `HDRP/Lit`. This is the standard PBR shader for HDRP.
4.  **Assign Textures**: Drag and drop your texture maps (Albedo, Normal, Metallic, Smoothness, AO) into their respective slots in the Material Inspector.
5.  **Adjust Properties**: Fine-tune properties like `Metallic` and `Smoothness` sliders if you don't have dedicated maps, or to enhance the material.

### Applying a PBR Material

1.  **Select the 3D Object** in your scene (e.g., your humanoid robot's parts, the ground plane).
2.  **In the Inspector window**, find the `Mesh Renderer` component.
3.  **Drag and Drop**: Drag your newly created PBR material from the Project window onto the `Material` slot in the `Mesh Renderer` component.

By applying PBR materials, you significantly enhance the visual realism of your simulated environment, making it more effective for HRI studies and visual AI training.

This concludes our chapter on using Unity for high-fidelity rendering and human-robot interaction.

