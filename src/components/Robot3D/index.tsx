import React, { useEffect, useRef } from 'react';
import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';

const Robot3D: React.FC = () => {
  const containerRef = useRef<HTMLDivElement>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const modelRef = useRef<THREE.Group | null>(null);
  const controlsRef = useRef<OrbitControls | null>(null);
  const animationRef = useRef<number | null>(null);

  useEffect(() => {
    if (!containerRef.current) return;

    // Initialize scene
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0a0a0f); // Match the theme background
    sceneRef.current = scene;

    // Initialize camera
    const camera = new THREE.PerspectiveCamera(
      45,
      containerRef.current.clientWidth / containerRef.current.clientHeight,
      0.1,
      1000
    );
    camera.position.set(0, 0, 5);
    cameraRef.current = camera;

    // Initialize renderer
    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setSize(containerRef.current.clientWidth, containerRef.current.clientHeight);
    renderer.setPixelRatio(window.devicePixelRatio);
    containerRef.current.appendChild(renderer.domElement);
    rendererRef.current = renderer;

    // Add lighting
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambientLight);

    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(1, 1, 1);
    scene.add(directionalLight);

    // Add point lights for the neon effect
    const pointLight1 = new THREE.PointLight(0x0043FF, 1, 10); // neon-blue
    pointLight1.position.set(2, 2, 2);
    scene.add(pointLight1);

    const pointLight2 = new THREE.PointLight(0xd17d3e, 0.5, 10); // neon-orange
    pointLight2.position.set(-2, -1, 2);
    scene.add(pointLight2);

    // Load the 3D model
    const loader = new GLTFLoader();
    loader.load(
      '/models/robot.glb', // Path to the 3D model
      (gltf) => {
        const model = gltf.scene;
        scene.add(model);
        modelRef.current = model;

        // Position the model in the center
        model.position.y = -0.5;

        // Add animation to the robot's hand/arm for waving
        animateModel(model);
      },
      undefined,
      (error) => {
        console.error('Error loading 3D model:', error);
        // Fallback: create a simple cube if model fails to load
        const geometry = new THREE.BoxGeometry(1, 1, 1);
        const material = new THREE.MeshPhongMaterial({ color: 0x0043FF });
        const cube = new THREE.Mesh(geometry, material);
        scene.add(cube);
        modelRef.current = cube;
      }
    );

    // Add orbit controls (for development, can be disabled for production)
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controlsRef.current = controls;

    // Handle window resize
    const handleResize = () => {
      if (!containerRef.current || !cameraRef.current || !rendererRef.current) return;
      cameraRef.current.aspect = containerRef.current.clientWidth / containerRef.current.clientHeight;
      cameraRef.current.updateProjectionMatrix();
      rendererRef.current.setSize(containerRef.current.clientWidth, containerRef.current.clientHeight);
    };

    window.addEventListener('resize', handleResize);

    // Animation function
    const animate = () => {
      animationRef.current = requestAnimationFrame(animate);

      if (controlsRef.current) {
        controlsRef.current.update();
      }

      if (rendererRef.current && cameraRef.current && sceneRef.current) {
        rendererRef.current.render(sceneRef.current, cameraRef.current);
      }
    };

    animate();

    // Cleanup function
    return () => {
      window.removeEventListener('resize', handleResize);

      if (animationRef.current) {
        cancelAnimationFrame(animationRef.current);
      }

      if (rendererRef.current) {
        rendererRef.current.dispose();
      }

      if (controlsRef.current) {
        controlsRef.current.dispose();
      }

      if (containerRef.current && rendererRef.current?.domElement) {
        containerRef.current.removeChild(rendererRef.current.domElement);
      }
    };
  }, []);

  // Function to animate the robot's waving motion
  const animateModel = (model: THREE.Group) => {
    // This function would animate specific parts of the robot model
    // For example, rotating the arm to create a waving motion
    // This is a simplified implementation - in a real scenario, you'd target specific bones/meshes
    let wavePhase = 0;

    const waveAnimation = () => {
      wavePhase += 0.02;

      // Example: animate a specific part of the model (like an arm)
      // This would depend on the actual structure of the loaded model
      // For now, we'll add a subtle rotation to simulate waving
      if (model) {
        // In a real implementation, you'd target specific parts of the robot
        // model.getObjectByName('arm')?.rotation.z = Math.sin(wavePhase) * 0.3;
      }

      requestAnimationFrame(waveAnimation);
    };

    waveAnimation();
  };

  return <div ref={containerRef} style={{ width: '100%', height: '300px' }} />;
};

export default Robot3D;