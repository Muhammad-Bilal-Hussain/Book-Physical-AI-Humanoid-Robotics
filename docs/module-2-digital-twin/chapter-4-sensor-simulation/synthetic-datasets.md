# Synthetic Dataset Generation Examples

Synthetic dataset generation in digital twin environments enables the creation of large, diverse, and perfectly-labeled datasets for training AI models in robotics. This chapter provides practical examples of synthetic dataset generation for various robotics applications.

## Overview

Synthetic datasets offer several advantages over real-world data collection:
- Unlimited data generation with perfect annotations
- Controlled environmental conditions
- Cost-effective and safe data collection
- Ability to generate edge cases difficult to reproduce physically

## Dataset Generation Pipelines

### Object Detection Dataset

#### Configuration Example
```python
# dataset_config.py
DATASET_CONFIG = {
    'name': 'Synthetic Indoor Objects',
    'classes': ['chair', 'table', 'robot', 'person', 'plant'],
    'image_size': (640, 480),
    'num_samples': 10000,
    'lighting_conditions': ['bright', 'dim', 'overcast', 'night'],
    'weather_conditions': ['clear', 'foggy'],
    'object_poses': 'random',
    'backgrounds': 'diverse_indoor_scenes'
}
```

#### Generation Script
```python
import numpy as np
import cv2
import json
from unity_simulation import UnityEnvironment

class SyntheticObjectDatasetGenerator:
    def __init__(self, config):
        self.config = config
        self.env = UnityEnvironment()
        self.annotations = []
        
    def generate_dataset(self):
        for i in range(self.config['num_samples']):
            # Set random environment conditions
            self.env.set_lighting(np.random.choice(self.config['lighting_conditions']))
            self.env.set_weather(np.random.choice(self.config['weather_conditions']))
            
            # Place objects randomly in scene
            objects = self.place_random_objects()
            
            # Capture image and annotations
            image = self.env.capture_rgb_image()
            depth = self.env.capture_depth_image()
            annotations = self.get_annotations(objects)
            
            # Save image and annotations
            self.save_sample(image, depth, annotations, i)
            
            # Reset scene for next sample
            self.env.reset_scene()
    
    def place_random_objects(self):
        objects = []
        num_objects = np.random.randint(1, 5)
        
        for _ in range(num_objects):
            obj_class = np.random.choice(self.config['classes'])
            position = self.random_position_in_scene()
            rotation = self.random_rotation()
            
            obj = self.env.spawn_object(obj_class, position, rotation)
            objects.append(obj)
        
        return objects
    
    def get_annotations(self, objects):
        annotations = []
        for obj in objects:
            bbox = self.env.get_bounding_box(obj)
            annotations.append({
                'class': obj.class_name,
                'bbox': bbox,
                'pose': obj.pose
            })
        return annotations
    
    def save_sample(self, image, depth, annotations, sample_id):
        # Save RGB image
        cv2.imwrite(f"images/{sample_id:06d}.png", image)
        
        # Save depth image
        cv2.imwrite(f"depth/{sample_id:06d}.png", depth)
        
        # Save annotations
        with open(f"annotations/{sample_id:06d}.json", 'w') as f:
            json.dump(annotations, f)
```

## Semantic Segmentation Dataset

### Unity Implementation
```csharp
using UnityEngine;
using System.Collections;
using System.IO;

public class SemanticSegmentationGenerator : MonoBehaviour
{
    public Camera rgbCamera;
    public Camera segmentationCamera;
    public string outputDirectory = "segmentation_dataset";
    
    private int sampleCount = 0;
    
    void Start()
    {
        // Create output directories
        Directory.CreateDirectory(Path.Combine(outputDirectory, "images"));
        Directory.CreateDirectory(Path.Combine(outputDirectory, "labels"));
    }
    
    public void GenerateSample()
    {
        // Capture RGB image
        Texture2D rgbImage = CaptureCameraImage(rgbCamera);
        SaveImage(rgbImage, Path.Combine(outputDirectory, "images", $"{sampleCount:D6}.png"));
        
        // Capture segmentation mask
        Texture2D segImage = CaptureCameraImage(segmentationCamera);
        SaveImage(segImage, Path.Combine(outputDirectory, "labels", $"{sampleCount:D6}.png"));
        
        sampleCount++;
    }
    
    Texture2D CaptureCameraImage(Camera cam)
    {
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = cam.targetTexture;
        cam.Render();
        
        Texture2D image = new Texture2D(cam.targetTexture.width, cam.targetTexture.height);
        image.ReadPixels(new Rect(0, 0, cam.targetTexture.width, cam.targetTexture.height), 0, 0);
        image.Apply();
        
        RenderTexture.active = currentRT;
        return image;
    }
    
    void SaveImage(Texture2D image, string path)
    {
        byte[] bytes = image.EncodeToPNG();
        File.WriteAllBytes(path, bytes);
        DestroyImmediate(image);
    }
}
```

## Depth Estimation Dataset

### Multi-modal Dataset Generation
```python
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

class DepthEstimationDataset:
    def __init__(self, unity_env):
        self.env = unity_env
        self.camera_intrinsics = self.env.get_camera_intrinsics()
        
    def generate_stereo_dataset(self):
        """Generate dataset for stereo depth estimation"""
        for i in range(10000):
            # Set random camera positions
            baseline = np.random.uniform(0.1, 0.3)  # 10-30cm baseline
            left_pos = self.env.get_random_camera_position()
            right_pos = left_pos.copy()
            right_pos[0] -= baseline  # Move right camera to the right
            
            # Set both cameras to look at the same point
            look_at_point = self.env.get_random_look_at_point()
            self.env.set_camera_pose('left', left_pos, look_at_point)
            self.env.set_camera_pose('right', right_pos, look_at_point)
            
            # Capture stereo pair
            left_img = self.env.capture_rgb_image('left')
            right_img = self.env.capture_rgb_image('right')
            depth_map = self.env.capture_ground_truth_depth()
            
            # Save stereo pair and depth
            self.save_stereo_pair(left_img, right_img, depth_map, i)
    
    def generate_monocular_dataset(self):
        """Generate dataset for monocular depth estimation"""
        for i in range(10000):
            # Random camera pose
            pos = self.env.get_random_camera_position()
            rot = R.random().as_euler('xyz')
            self.env.set_camera_pose('main', pos, rot)
            
            # Add random objects to scene
            self.env.randomize_scene()
            
            # Capture image and depth
            rgb_img = self.env.capture_rgb_image('main')
            depth_img = self.env.capture_ground_truth_depth('main')
            
            # Save sample
            self.save_monocular_sample(rgb_img, depth_img, i)
```

## LiDAR Dataset Generation

### Point Cloud Dataset
```python
import numpy as np
import open3d as o3d

class LiDARDatasetGenerator:
    def __init__(self, gazebo_env):
        self.gazebo = gazebo_env
        
    def generate_detection_dataset(self):
        """Generate point cloud dataset for object detection"""
        for i in range(5000):
            # Randomize scene with objects
            self.gazebo.randomize_scene_objects()
            
            # Get ground truth poses
            gt_poses = self.gazebo.get_object_poses()
            
            # Capture LiDAR scan
            point_cloud = self.gazebo.get_lidar_scan()
            
            # Generate annotations
            annotations = self.generate_bounding_boxes(gt_poses, point_cloud)
            
            # Save point cloud and annotations
            self.save_lidar_sample(point_cloud, annotations, i)
    
    def generate_bounding_boxes(self, gt_poses, point_cloud):
        """Generate 3D bounding boxes from ground truth poses"""
        boxes = []
        for obj_pose in gt_poses:
            # Create oriented bounding box from object pose and dimensions
            bbox = self.create_oriented_bbox(obj_pose)
            boxes.append(bbox)
        return boxes
    
    def save_lidar_sample(self, point_cloud, annotations, sample_id):
        """Save point cloud and annotations"""
        # Save point cloud as PCD file
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(point_cloud)
        o3d.io.write_point_cloud(f"pointclouds/{sample_id:06d}.pcd", pcd)
        
        # Save annotations
        np.save(f"labels/{sample_id:06d}_labels.npy", annotations)
```

## Domain Randomization Techniques

### Lighting Variation
```python
def randomize_lighting(unity_env):
    # Randomize directional light
    sun = unity_env.get_light('Sun')
    sun.color = np.random.uniform(0.8, 1.0, 3)  # Color temperature
    sun.intensity = np.random.uniform(0.5, 2.0)  # Intensity
    
    # Randomize ambient lighting
    unity_env.set_ambient_light(
        np.random.uniform(0.1, 0.3),  # Intensity
        np.random.uniform(0.4, 0.8, 3)  # Color
    )
    
    # Add random point lights
    for i in range(np.random.randint(0, 3)):
        pos = np.random.uniform(-5, 5, 3)
        color = np.random.uniform(0.5, 1.0, 3)
        intensity = np.random.uniform(0.5, 1.5)
        unity_env.add_point_light(pos, color, intensity)
```

### Texture and Material Randomization
```python
def randomize_materials(unity_env):
    # Randomize textures for objects
    for obj in unity_env.get_all_objects():
        if hasattr(obj, 'renderer'):
            # Randomize color
            obj.renderer.material.color = np.random.uniform(0, 1, 3)
            
            # Randomize roughness and metallic properties
            obj.renderer.material.SetFloat("_Metallic", np.random.uniform(0, 1))
            obj.renderer.material.SetFloat("_Smoothness", np.random.uniform(0, 1))
            
            # Apply random texture from a set of options
            texture = np.random.choice(available_textures)
            obj.renderer.material.mainTexture = texture
```

## Quality Assurance

### Dataset Validation
```python
def validate_dataset(dataset_path):
    """Validate generated dataset for completeness and quality"""
    image_dir = os.path.join(dataset_path, 'images')
    label_dir = os.path.join(dataset_path, 'labels')
    
    # Check that all images have corresponding labels
    images = set([f for f in os.listdir(image_dir) if f.endswith('.png')])
    labels = set([f for f in os.listdir(label_dir) if f.endswith('.json')])
    
    missing_labels = images - labels
    missing_images = labels - images
    
    if missing_labels or missing_images:
        print(f"Missing labels for images: {missing_labels}")
        print(f"Missing images for labels: {missing_images}")
        return False
    
    # Validate annotation format
    for label_file in labels:
        with open(os.path.join(label_dir, label_file), 'r') as f:
            try:
                annotations = json.load(f)
                # Validate annotation structure
                assert 'objects' in annotations
                for obj in annotations['objects']:
                    assert 'bbox' in obj
                    assert 'class' in obj
            except (json.JSONDecodeError, AssertionError) as e:
                print(f"Invalid annotation in {label_file}: {e}")
                return False
    
    return True
```

## Best Practices

1. **Consistent Naming Conventions**: Use consistent naming for images, labels, and metadata files
2. **Metadata Documentation**: Include comprehensive metadata about generation parameters
3. **Quality Control**: Implement validation checks to ensure dataset integrity
4. **Diversity**: Ensure dataset covers the full range of expected operating conditions
5. **Realism**: Balance synthetic nature with realistic appearance for effective sim-to-real transfer

## Conclusion

Synthetic dataset generation in digital twin environments provides powerful capabilities for robotics AI development, enabling the creation of large, diverse, and perfectly-labeled datasets that accelerate the development of robust perception systems.