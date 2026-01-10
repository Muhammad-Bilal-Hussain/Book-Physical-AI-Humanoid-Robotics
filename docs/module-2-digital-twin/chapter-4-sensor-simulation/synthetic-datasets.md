# Synthetic Dataset Generation Examples

## Introduction to Synthetic Dataset Generation

Synthetic dataset generation is the process of creating artificial data that mimics real-world sensor data for training AI perception systems. This chapter provides practical examples of how to generate synthetic datasets for various robotic sensors, including LiDAR, cameras, and IMUs. These examples demonstrate techniques for creating diverse, annotated datasets that can be used to train robust perception algorithms.

## LiDAR Dataset Generation

### Point Cloud Dataset for Object Detection

#### Example 1: Urban Scene Generation

Creating a synthetic dataset for urban LiDAR object detection:

```python
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
import random

class UrbanLiDARGenerator:
    def __init__(self):
        self.scene_bounds = {
            'x': (-50, 50),
            'y': (-50, 50),
            'z': (0, 10)
        }
        
    def generate_car(self, position, orientation=None):
        """Generate a synthetic car point cloud"""
        if orientation is None:
            orientation = [0, 0, 0]  # No rotation
        
        # Car dimensions (length, width, height)
        length, width, height = 4.0, 1.8, 1.5
        
        # Create points for a car
        x = np.random.uniform(-length/2, length/2, 500)
        y = np.random.uniform(-width/2, width/2, 500)
        z = np.random.uniform(0, height, 500)
        
        # Stack points
        points = np.column_stack((x, y, z))
        
        # Apply rotation
        rot = R.from_euler('xyz', orientation, degrees=True)
        points = rot.apply(points)
        
        # Apply translation
        points += position
        
        # Add some noise
        noise = np.random.normal(0, 0.02, points.shape)
        points += noise
        
        return points, 'car'
    
    def generate_pedestrian(self, position):
        """Generate a synthetic pedestrian point cloud"""
        # Create a simple human-like shape
        height = 1.7
        width = 0.5
        
        # Head
        head_radius = 0.15
        head_points = []
        for _ in range(100):
            r = np.random.uniform(0, head_radius)
            theta = np.random.uniform(0, 2*np.pi)
            phi = np.random.uniform(0, np.pi)
            x = r * np.sin(phi) * np.cos(theta)
            y = r * np.sin(phi) * np.sin(theta)
            z = r * np.cos(phi) + height - head_radius
            head_points.append([x, y, z])
        
        # Body (cylinder)
        body_points = []
        for _ in range(200):
            r = np.random.uniform(0, width/2)
            theta = np.random.uniform(0, 2*np.pi)
            x = r * np.cos(theta)
            y = r * np.sin(theta)
            z = np.random.uniform(0.2, height - head_radius)
            body_points.append([x, y, z])
        
        points = np.array(head_points + body_points)
        points += position
        
        # Add noise
        noise = np.random.normal(0, 0.01, points.shape)
        points += noise
        
        return points, 'pedestrian'
    
    def generate_scene(self, num_cars=5, num_pedestrians=10):
        """Generate a complete urban scene"""
        all_points = []
        all_labels = []
        
        # Generate cars
        for i in range(num_cars):
            x = np.random.uniform(self.scene_bounds['x'][0], 
                                 self.scene_bounds['x'][1])
            y = np.random.uniform(self.scene_bounds['y'][0], 
                                 self.scene_bounds['y'][1])
            z = 0  # Ground level
            position = [x, y, z]
            
            car_points, label = self.generate_car(position)
            all_points.append(car_points)
            all_labels.extend([label] * len(car_points))
        
        # Generate pedestrians
        for i in range(num_pedestrians):
            x = np.random.uniform(self.scene_bounds['x'][0], 
                                 self.scene_bounds['x'][1])
            y = np.random.uniform(self.scene_bounds['y'][0], 
                                 self.scene_bounds['y'][1])
            z = 0  # Ground level
            position = [x, y, z]
            
            ped_points, label = self.generate_pedestrian(position)
            all_points.append(ped_points)
            all_labels.extend([label] * len(ped_points))
        
        # Combine all points
        combined_points = np.vstack(all_points)
        
        # Add ground plane
        ground_x = np.random.uniform(self.scene_bounds['x'][0], 
                                    self.scene_bounds['x'][1], 1000)
        ground_y = np.random.uniform(self.scene_bounds['y'][0], 
                                    self.scene_bounds['y'][1], 1000)
        ground_z = np.random.uniform(-0.1, 0.1, 1000)  # Slightly below ground
        ground_points = np.column_stack((ground_x, ground_y, ground_z))
        
        combined_points = np.vstack([combined_points, ground_points])
        all_labels.extend(['ground'] * len(ground_points))
        
        return combined_points, all_labels

# Example usage
generator = UrbanLiDARGenerator()
points, labels = generator.generate_scene(num_cars=3, num_pedestrians=5)

# Save the dataset
np.savez('urban_lidar_dataset.npz', points=points, labels=labels)
print(f"Generated dataset with {len(points)} points")
```

#### Example 2: Indoor Navigation Dataset

Creating a synthetic dataset for indoor LiDAR navigation:

```python
import numpy as np
import random

class IndoorLiDARGenerator:
    def __init__(self):
        self.room_bounds = {
            'x': (0, 10),
            'y': (0, 8),
            'z': (0, 3)
        }
        
    def generate_room_layout(self):
        """Generate walls and basic room structure"""
        points = []
        
        # Generate floor
        floor_x = np.random.uniform(self.room_bounds['x'][0], 
                                   self.room_bounds['x'][1], 2000)
        floor_y = np.random.uniform(self.room_bounds['y'][0], 
                                   self.room_bounds['y'][1], 2000)
        floor_z = np.random.uniform(-0.05, 0.05, 2000)  # Floor with slight variation
        floor_points = np.column_stack((floor_x, floor_y, floor_z))
        points.extend(floor_points)
        
        # Generate ceiling
        ceil_x = np.random.uniform(self.room_bounds['x'][0], 
                                  self.room_bounds['x'][1], 2000)
        ceil_y = np.random.uniform(self.room_bounds['y'][0], 
                                  self.room_bounds['y'][1], 2000)
        ceil_z = np.random.uniform(self.room_bounds['z'][1]-0.05, 
                                  self.room_bounds['z'][1]+0.05, 2000)
        ceil_points = np.column_stack((ceil_x, ceil_y, ceil_z))
        points.extend(ceil_points)
        
        # Generate walls
        # Wall 1 (x = min_x)
        wall1_x = np.random.uniform(self.room_bounds['x'][0]-0.05, 
                                   self.room_bounds['x'][0]+0.05, 1000)
        wall1_y = np.random.uniform(self.room_bounds['y'][0], 
                                   self.room_bounds['y'][1], 1000)
        wall1_z = np.random.uniform(0, self.room_bounds['z'][1], 1000)
        wall1_points = np.column_stack((wall1_x, wall1_y, wall1_z))
        points.extend(wall1_points)
        
        # Wall 2 (x = max_x)
        wall2_x = np.random.uniform(self.room_bounds['x'][1]-0.05, 
                                   self.room_bounds['x'][1]+0.05, 1000)
        wall2_y = np.random.uniform(self.room_bounds['y'][0], 
                                   self.room_bounds['y'][1], 1000)
        wall2_z = np.random.uniform(0, self.room_bounds['z'][1], 1000)
        wall2_points = np.column_stack((wall2_x, wall2_y, wall2_z))
        points.extend(wall2_points)
        
        # Wall 3 (y = min_y)
        wall3_x = np.random.uniform(self.room_bounds['x'][0], 
                                   self.room_bounds['x'][1], 1000)
        wall3_y = np.random.uniform(self.room_bounds['y'][0]-0.05, 
                                   self.room_bounds['y'][0]+0.05, 1000)
        wall3_z = np.random.uniform(0, self.room_bounds['z'][1], 1000)
        wall3_points = np.column_stack((wall3_x, wall3_y, wall3_z))
        points.extend(wall3_points)
        
        # Wall 4 (y = max_y)
        wall4_x = np.random.uniform(self.room_bounds['x'][0], 
                                   self.room_bounds['x'][1], 1000)
        wall4_y = np.random.uniform(self.room_bounds['y'][1]-0.05, 
                                   self.room_bounds['y'][1]+0.05, 1000)
        wall4_z = np.random.uniform(0, self.room_bounds['z'][1], 1000)
        wall4_points = np.column_stack((wall4_x, wall4_y, wall4_z))
        points.extend(wall4_points)
        
        return np.array(points)
    
    def generate_furniture(self, room_points):
        """Add furniture to the room"""
        furniture_points = []
        
        # Generate tables
        for _ in range(3):
            # Table top
            table_x = np.random.uniform(1, 8, 200)
            table_y = np.random.uniform(1, 6, 200)
            table_z = np.random.uniform(0.7, 0.75, 200)  # Standard table height
            table_top = np.column_stack((table_x, table_y, table_z))
            furniture_points.extend(table_top)
            
            # Table legs
            for leg_x_offset in [-0.4, 0.4]:
                for leg_y_offset in [-0.3, 0.3]:
                    leg_x = table_x[:50] + leg_x_offset
                    leg_y = table_y[:50] + leg_y_offset
                    leg_z = np.random.uniform(0.1, 0.7, 50)
                    leg_points = np.column_stack((leg_x, leg_y, leg_z))
                    furniture_points.extend(leg_points)
        
        # Generate chairs
        for _ in range(5):
            # Chair seat
            chair_x = np.random.uniform(1, 8, 100)
            chair_y = np.random.uniform(1, 6, 100)
            chair_z = np.random.uniform(0.45, 0.5, 100)
            chair_seat = np.column_stack((chair_x, chair_y, chair_z))
            furniture_points.extend(chair_seat)
            
            # Chair back
            back_x = chair_x[:50]
            back_y = chair_y[:50]
            back_z = np.random.uniform(0.5, 0.9, 50)
            chair_back = np.column_stack((back_x, back_y, back_z))
            furniture_points.extend(chair_back)
        
        return np.array(furniture_points)
    
    def generate_dataset(self, num_scenes=10):
        """Generate multiple indoor scenes"""
        all_scenes = []
        
        for scene_id in range(num_scenes):
            room_points = self.generate_room_layout()
            furniture_points = self.generate_furniture(room_points)
            
            # Combine room and furniture
            scene_points = np.vstack([room_points, furniture_points])
            
            # Add some noise to simulate sensor imperfections
            noise = np.random.normal(0, 0.01, scene_points.shape)
            scene_points += noise
            
            all_scenes.append(scene_points)
        
        return all_scenes

# Example usage
indoor_gen = IndoorLiDARGenerator()
indoor_scenes = indoor_gen.generate_dataset(num_scenes=5)

# Save the dataset
for i, scene in enumerate(indoor_scenes):
    np.save(f'indoor_scene_{i}.npy', scene)
    
print(f"Generated {len(indoor_scenes)} indoor scenes")
```

## Camera Dataset Generation

### RGB Dataset for Object Recognition

#### Example 3: Synthetic Object Dataset

Creating a synthetic RGB dataset for object recognition:

```python
import cv2
import numpy as np
import random
from PIL import Image, ImageDraw, ImageFont

class SyntheticRGBGenerator:
    def __init__(self):
        self.object_shapes = ['circle', 'rectangle', 'triangle', 'star']
        self.colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), 
                      (255, 0, 255), (0, 255, 255), (128, 0, 128)]
        self.backgrounds = ['plain', 'grid', 'textured']
    
    def generate_background(self, width=640, height=480, bg_type='plain'):
        """Generate different types of backgrounds"""
        if bg_type == 'plain':
            bg = np.random.randint(200, 255, (height, width, 3), dtype=np.uint8)
        elif bg_type == 'grid':
            bg = np.random.randint(200, 255, (height, width, 3), dtype=np.uint8)
            # Add grid pattern
            for i in range(0, height, 30):
                cv2.line(bg, (0, i), (width, i), (180, 180, 180), 1)
            for i in range(0, width, 30):
                cv2.line(bg, (i, 0), (i, height), (180, 180, 180), 1)
        elif bg_type == 'textured':
            bg = np.random.randint(180, 240, (height, width, 3), dtype=np.uint8)
            # Add some texture
            for _ in range(100):
                x = random.randint(0, width-1)
                y = random.randint(0, height-1)
                color = tuple(random.randint(150, 200) for _ in range(3))
                cv2.circle(bg, (x, y), 1, color, -1)
        
        return bg
    
    def draw_shape(self, img, shape, x, y, size, color):
        """Draw a shape on the image"""
        overlay = img.copy()
        
        if shape == 'circle':
            cv2.circle(overlay, (x, y), size, color, -1)
        elif shape == 'rectangle':
            pt1 = (x - size, y - size)
            pt2 = (x + size, y + size)
            cv2.rectangle(overlay, pt1, pt2, color, -1)
        elif shape == 'triangle':
            points = np.array([
                [x, y - size],
                [x - size, y + size],
                [x + size, y + size]
            ], np.int32)
            cv2.fillPoly(overlay, [points], color)
        elif shape == 'star':
            # Draw a simple 5-pointed star
            points = []
            for i in range(10):
                angle = i * np.pi / 5
                radius = size if i % 2 == 0 else size // 2
                px = x + int(radius * np.cos(angle - np.pi/2))
                py = y + int(radius * np.sin(angle - np.pi/2))
                points.append([px, py])
            points = np.array(points, np.int32)
            cv2.fillPoly(overlay, [points], color)
        
        # Apply some transparency
        alpha = 0.8
        img = cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0)
        return img
    
    def generate_image(self, width=640, height=480):
        """Generate a synthetic image with objects"""
        # Select background type
        bg_type = random.choice(self.backgrounds)
        img = self.generate_background(width, height, bg_type)
        
        # Add multiple objects
        num_objects = random.randint(3, 8)
        annotations = []
        
        for _ in range(num_objects):
            shape = random.choice(self.object_shapes)
            color = random.choice(self.colors)
            size = random.randint(20, 60)
            
            # Position object randomly but avoid edges
            x = random.randint(size, width - size)
            y = random.randint(size, height - size)
            
            # Draw the object
            img = self.draw_shape(img, shape, x, y, size, color)
            
            # Add annotation
            annotations.append({
                'shape': shape,
                'color': color,
                'bbox': [x-size, y-size, x+size, y+size],
                'center': [x, y]
            })
        
        # Add some noise to simulate real camera
        noise = np.random.normal(0, 5, img.shape).astype(np.uint8)
        img = cv2.add(img, noise)
        
        # Apply slight blur to simulate camera focus
        img = cv2.GaussianBlur(img, (3, 3), 0)
        
        return img, annotations
    
    def generate_dataset(self, num_images=100, width=640, height=480):
        """Generate a complete dataset"""
        images = []
        annotations = []
        
        for i in range(num_images):
            img, ann = self.generate_image(width, height)
            images.append(img)
            annotations.append(ann)
            
            # Save image
            cv2.imwrite(f'synthetic_rgb_{i:03d}.png', img)
            
            # Save annotation
            with open(f'synthetic_rgb_{i:03d}.txt', 'w') as f:
                for obj in ann:
                    f.write(f"{obj['shape']} {obj['bbox'][0]} {obj['bbox'][1]} {obj['bbox'][2]} {obj['bbox'][3]}\n")
        
        return images, annotations

# Example usage
rgb_gen = SyntheticRGBGenerator()
images, annotations = rgb_gen.generate_dataset(num_images=10)

print(f"Generated {len(images)} synthetic RGB images")
```

### Depth Dataset Generation

#### Example 4: Synthetic Depth Dataset

Creating a synthetic depth dataset for depth estimation:

```python
import numpy as np
import cv2
from PIL import Image

class SyntheticDepthGenerator:
    def __init__(self):
        self.scene_types = ['indoor', 'outdoor', 'cluttered']
    
    def generate_depth_map(self, width=640, height=480, scene_type='indoor'):
        """Generate a synthetic depth map"""
        depth_map = np.zeros((height, width), dtype=np.float32)
        
        if scene_type == 'indoor':
            # Create a simple indoor scene with objects at different depths
            # Background wall
            depth_map[:, :] = 5.0  # 5 meters to back wall
            
            # Add objects at different depths
            # Table in the middle
            center_x, center_y = width // 2, height // 2
            table_size_x, table_size_y = 100, 80
            table_depth = 2.0  # 2 meters from camera
            depth_map[center_y-table_size_y//2:center_y+table_size_y//2,
                     center_x-table_size_x//2:center_x+table_size_x//2] = table_depth
            
            # Chair in front of table
            chair_depth = 1.5  # 1.5 meters from camera
            depth_map[center_y+table_size_y//2:center_y+table_size_y//2+60,
                     center_x-40:center_x+40] = chair_depth
            
            # Objects on table
            for _ in range(5):
                obj_x = np.random.randint(center_x-table_size_x//2+10, 
                                         center_x+table_size_x//2-10)
                obj_y = np.random.randint(center_y-table_size_y//2+10, 
                                         center_y+table_size_y//2-10)
                obj_size = np.random.randint(10, 20)
                obj_depth = table_depth - 0.1  # Slightly in front of table
                depth_map[obj_y-obj_size:obj_y+obj_size, 
                         obj_x-obj_size:obj_x+obj_size] = obj_depth
        
        elif scene_type == 'outdoor':
            # Create a simple outdoor scene
            # Ground plane with varying depth
            for y in range(height):
                depth_map[y, :] = 1.0 + (y / height) * 10.0  # Ground slopes away
            
            # Add a building in the background
            building_width = 200
            building_start = (width - building_width) // 2
            building_depth = 8.0
            depth_map[height//3:, building_start:building_start+building_width] = building_depth
            
            # Add trees
            for _ in range(3):
                tree_x = np.random.randint(50, width-50)
                tree_width = 30
                tree_depth = np.random.uniform(3.0, 6.0)
                depth_map[height//2:, tree_x-tree_width//2:tree_x+tree_width//2] = tree_depth
        
        elif scene_type == 'cluttered':
            # Create a cluttered scene with many objects
            depth_map[:, :] = 10.0  # Far background
            
            # Add multiple objects at different depths
            for _ in range(15):
                obj_x = np.random.randint(20, width-20)
                obj_y = np.random.randint(20, height-20)
                obj_size = np.random.randint(15, 50)
                obj_depth = np.random.uniform(0.5, 8.0)
                
                # Create a circular object
                y, x = np.ogrid[:height, :width]
                mask = (x - obj_x)**2 + (y - obj_y)**2 <= obj_size**2
                depth_map[mask] = obj_depth
        
        # Add some noise to make it more realistic
        noise = np.random.normal(0, 0.02, depth_map.shape).astype(np.float32)
        depth_map += noise
        
        # Ensure depth values are positive
        depth_map = np.maximum(depth_map, 0.1)
        
        return depth_map
    
    def generate_multiview_depth(self, width=640, height=480):
        """Generate depth maps from multiple viewpoints"""
        # Generate base depth map
        base_depth = self.generate_depth_map(width, height)
        
        # Generate depth maps from slightly different viewpoints
        viewpoints = []
        for i in range(5):  # 5 different viewpoints
            # Simulate camera movement
            dx = np.random.uniform(-0.1, 0.1)  # Small translation
            dy = np.random.uniform(-0.1, 0.1)
            
            # Create transformation matrix for small translation
            M = np.float32([[1, 0, dx * width], [0, 1, dy * height]])
            
            # Apply transformation
            transformed_depth = cv2.warpAffine(base_depth, M, (width, height), 
                                              flags=cv2.INTER_LINEAR, 
                                              borderMode=cv2.BORDER_REFLECT)
            
            viewpoints.append(transformed_depth)
        
        return viewpoints
    
    def generate_dataset(self, num_scenes=20):
        """Generate a complete depth dataset"""
        for i in range(num_scenes):
            scene_type = random.choice(self.scene_types)
            depth_map = self.generate_depth_map(scene_type=scene_type)
            
            # Convert to 16-bit format for saving (in millimeters)
            depth_mm = (depth_map * 1000).astype(np.uint16)
            
            # Save depth map
            depth_img = Image.fromarray(depth_mm)
            depth_img.save(f'depth_map_{i:03d}.png')
            
            # Also save as numpy array
            np.save(f'depth_map_{i:03d}.npy', depth_map)
        
        print(f"Generated {num_scenes} synthetic depth maps")

# Example usage
depth_gen = SyntheticDepthGenerator()
depth_gen.generate_dataset(num_scenes=5)
```

## IMU Dataset Generation

### Example 5: Synthetic IMU Dataset

Creating a synthetic IMU dataset for motion analysis:

```python
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

class SyntheticIMUMotionGenerator:
    def __init__(self, sample_rate=100):  # 100 Hz sample rate
        self.sample_rate = sample_rate
        self.dt = 1.0 / sample_rate
    
    def generate_linear_motion(self, duration=10.0, acceleration_profile='constant'):
        """Generate linear motion with known acceleration"""
        num_samples = int(duration * self.sample_rate)
        time = np.linspace(0, duration, num_samples)
        
        if acceleration_profile == 'constant':
            # Constant acceleration
            true_acc = np.array([0.5, 0.0, -9.8])  # Including gravity
            acceleration = np.tile(true_acc, (num_samples, 1))
            
        elif acceleration_profile == 'sine':
            # Sine wave acceleration
            freq = 0.5  # 0.5 Hz
            amplitude = 2.0
            x_acc = amplitude * np.sin(2 * np.pi * freq * time)
            y_acc = amplitude * np.cos(2 * np.pi * freq * time)
            z_acc = -9.8 + 0.5 * np.sin(2 * np.pi * freq * time * 0.7)  # Gravity + small variation
            acceleration = np.column_stack((x_acc, y_acc, z_acc))
        
        # Generate angular velocity (should be zero for pure linear motion)
        angular_velocity = np.zeros((num_samples, 3))
        
        return acceleration, angular_velocity, time
    
    def generate_rotational_motion(self, duration=10.0):
        """Generate rotational motion"""
        num_samples = int(duration * self.sample_rate)
        time = np.linspace(0, duration, num_samples)
        
        # Define rotation profile
        omega_x = 0.5 * np.sin(0.3 * time)  # Roll
        omega_y = 0.3 * np.cos(0.4 * time)  # Pitch  
        omega_z = 0.2 * np.sin(0.5 * time)  # Yaw
        
        angular_velocity = np.column_stack((omega_x, omega_y, omega_z))
        
        # Calculate orientation by integrating angular velocity
        orientation = np.zeros((num_samples, 4))  # Quaternion
        current_quat = np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion
        
        for i in range(1, num_samples):
            # Convert angular velocity to quaternion derivative
            omega = angular_velocity[i-1]
            omega_norm = np.linalg.norm(omega)
            
            if omega_norm > 1e-6:  # Avoid division by zero
                axis = omega / omega_norm
                angle = omega_norm * self.dt
                
                # Create rotation quaternion
                dq = np.array([
                    np.cos(angle/2),
                    axis[0] * np.sin(angle/2),
                    axis[1] * np.sin(angle/2), 
                    axis[2] * np.sin(angle/2)
                ])
                
                # Update orientation
                current_quat = self.quat_multiply(dq, current_quat)
                current_quat = current_quat / np.linalg.norm(current_quat)
            
            orientation[i] = current_quat
        
        # Calculate linear acceleration from rotation (centripetal and tangential)
        acceleration = np.zeros((num_samples, 3))
        
        # For simplicity, assume a point mass at distance r from rotation center
        r = 0.5  # 0.5m from center
        for i in range(num_samples):
            omega = angular_velocity[i]
            # Simplified model: centripetal acceleration = omega^2 * r
            omega_mag = np.linalg.norm(omega)
            if omega_mag > 1e-6:
                # Centripetal acceleration direction is perpendicular to rotation axis and position
                # For simplicity, we'll just use a simplified model
                acc_mag = omega_mag**2 * r
                # Direction is perpendicular to rotation axis
                acc_dir = np.array([-omega[1], omega[0], 0])  # Perpendicular in xy-plane
                if np.linalg.norm(acc_dir) > 1e-6:
                    acc_dir = acc_dir / np.linalg.norm(acc_dir)
                    acceleration[i] = acc_dir * acc_mag
            # Add gravity
            acceleration[i, 2] -= 9.81
        
        return acceleration, angular_velocity, time, orientation
    
    def quat_multiply(self, q1, q2):
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        
        return np.array([w, x, y, z])
    
    def add_imu_noise(self, true_acc, true_gyro, noise_params=None):
        """Add realistic IMU noise to the signals"""
        if noise_params is None:
            # Default noise parameters (typical for a good IMU)
            noise_params = {
                'acc_white': 0.017,      # m/s^2 / sqrt(Hz) (ADIS16448 typical)
                'acc_bias_instability': 25e-6 * 9.81,  # m/s^2
                'gyro_white': 0.0035,    # rad/s / sqrt(Hz)
                'gyro_bias_instability': 3.5e-5,       # rad/s
            }
        
        num_samples = len(true_acc)
        
        # Generate noise for accelerometers
        acc_white_noise = np.random.normal(0, noise_params['acc_white'] * np.sqrt(self.sample_rate), 
                                          (num_samples, 3))
        # Add bias instability (random walk)
        acc_bias = np.random.normal(0, noise_params['acc_bias_instability'], (num_samples, 3))
        acc_bias = np.cumsum(acc_bias, axis=0)  # Random walk
        
        # Generate noise for gyroscopes
        gyro_white_noise = np.random.normal(0, noise_params['gyro_white'] * np.sqrt(self.sample_rate), 
                                           (num_samples, 3))
        # Add bias instability (random walk)
        gyro_bias = np.random.normal(0, noise_params['gyro_bias_instability'], (num_samples, 3))
        gyro_bias = np.cumsum(gyro_bias, axis=0)  # Random walk
        
        # Add noise to true signals
        noisy_acc = true_acc + acc_white_noise + acc_bias
        noisy_gyro = true_gyro + gyro_white_noise + gyro_bias
        
        return noisy_acc, noisy_gyro
    
    def generate_dataset(self, num_trajectories=5):
        """Generate multiple IMU trajectories"""
        for traj_id in range(num_trajectories):
            print(f"Generating trajectory {traj_id+1}/{num_trajectories}")
            
            # Generate different types of motion
            motion_type = np.random.choice(['linear', 'rotational'])
            
            if motion_type == 'linear':
                acc, gyro, time = self.generate_linear_motion(duration=10.0, 
                                                            acceleration_profile='sine')
            else:  # rotational
                acc, gyro, time, orientation = self.generate_rotational_motion(duration=10.0)
            
            # Add realistic IMU noise
            noisy_acc, noisy_gyro = self.add_imu_noise(acc, gyro)
            
            # Save the data
            data_dict = {
                'time': time,
                'true_acceleration': acc,
                'noisy_acceleration': noisy_acc,
                'true_angular_velocity': gyro,
                'noisy_angular_velocity': noisy_gyro,
                'motion_type': motion_type
            }
            
            if motion_type == 'rotational':
                data_dict['orientation'] = orientation
            
            np.savez(f'imu_trajectory_{traj_id:02d}.npz', **data_dict)
        
        print(f"Generated {num_trajectories} synthetic IMU trajectories")

# Example usage
imu_gen = SyntheticIMUMotionGenerator(sample_rate=100)
imu_gen.generate_dataset(num_trajectories=3)
```

## Multi-Sensor Fusion Dataset

### Example 6: LiDAR-Camera Fusion Dataset

Creating a synthetic dataset that combines LiDAR and camera data:

```python
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

class MultiSensorFusionGenerator:
    def __init__(self):
        # Camera intrinsic parameters
        self.fx = 554.0
        self.fy = 554.0
        self.cx = 320.0
        self.cy = 240.0
        
        # Camera-LiDAR extrinsic calibration (rotation and translation)
        # This represents the transformation from LiDAR frame to camera frame
        self.R_lidar_to_cam = R.from_euler('xyz', [0, 0, np.pi/2]).as_matrix()  # 90-degree rotation
        self.t_lidar_to_cam = np.array([0.1, 0.05, 0.05])  # 10cm x, 5cm y, 5cm z offset
    
    def project_lidar_to_camera(self, points_3d):
        """Project 3D LiDAR points to 2D camera image coordinates"""
        # Transform points from LiDAR frame to camera frame
        points_cam = (self.R_lidar_to_cam @ points_3d.T).T + self.t_lidar_to_cam
        
        # Remove points behind the camera
        valid_idx = points_cam[:, 2] > 0
        points_cam = points_cam[valid_idx]
        
        # Project to 2D image coordinates
        x = points_cam[:, 0] * self.fx / points_cam[:, 2] + self.cx
        y = points_cam[:, 1] * self.fy / points_cam[:, 2] + self.cy
        
        # Create image coordinates and depth
        u = x.astype(int)
        v = y.astype(int)
        depth = points_cam[:, 2]
        
        # Filter points within image bounds
        valid_coords = (u >= 0) & (u < 640) & (v >= 0) & (v < 480)
        
        return {
            'u': u[valid_coords],
            'v': v[valid_coords], 
            'depth': depth[valid_coords],
            'points_3d': points_cam[valid_coords]
        }
    
    def generate_fusion_scene(self, num_objects=5):
        """Generate a scene with both LiDAR and camera data"""
        # Generate LiDAR point cloud
        lidar_gen = UrbanLiDARGenerator()
        points_3d, labels = lidar_gen.generate_scene(num_cars=2, num_pedestrians=3)
        
        # Project LiDAR points to camera
        projection = self.project_lidar_to_camera(points_3d)
        
        # Generate corresponding camera image
        # Create a simple background
        img = np.random.randint(200, 255, (480, 640, 3), dtype=np.uint8)
        
        # Draw projected LiDAR points on the image
        for u, v, depth in zip(projection['u'], projection['v'], projection['depth']):
            # Color based on depth
            color_intensity = int(255 * (1 - min(depth / 50.0, 1.0)))
            color = (color_intensity, 0, 255 - color_intensity)  # Blue-red gradient
            
            # Draw point
            cv2.circle(img, (int(u), int(v)), 2, color, -1)
        
        # Add some noise to the image
        noise = np.random.normal(0, 10, img.shape).astype(np.uint8)
        img = cv2.add(img, noise)
        
        return {
            'lidar_points': points_3d,
            'lidar_labels': labels,
            'camera_image': img,
            'projected_points': projection,
            'calibration': {
                'intrinsic': np.array([[self.fx, 0, self.cx],
                                      [0, self.fy, self.cy], 
                                      [0, 0, 1]]),
                'extrinsic_rot': self.R_lidar_to_cam,
                'extrinsic_trans': self.t_lidar_to_cam
            }
        }
    
    def generate_fusion_dataset(self, num_scenes=10):
        """Generate a multi-sensor fusion dataset"""
        for i in range(num_scenes):
            scene_data = self.generate_fusion_scene()
            
            # Save LiDAR data
            np.savez(f'fusion_lidar_{i:03d}.npz',
                    points=scene_data['lidar_points'],
                    labels=scene_data['lidar_labels'])
            
            # Save camera data
            cv2.imwrite(f'fusion_camera_{i:03d}.png', scene_data['camera_image'])
            
            # Save projected points
            np.savez(f'fusion_projection_{i:03d}.npz',
                    u=scene_data['projected_points']['u'],
                    v=scene_data['projected_points']['v'],
                    depth=scene_data['projected_points']['depth'])
            
            # Save calibration
            np.savez(f'fusion_calibration_{i:03d}.npz',
                    intrinsic=scene_data['calibration']['intrinsic'],
                    extrinsic_rot=scene_data['calibration']['extrinsic_rot'],
                    extrinsic_trans=scene_data['calibration']['extrinsic_trans'])
        
        print(f"Generated {num_scenes} multi-sensor fusion scenes")

# Example usage
fusion_gen = MultiSensorFusionGenerator()
fusion_gen.generate_fusion_dataset(num_scenes=5)
```

## Dataset Validation and Quality Assessment

### Example 7: Dataset Validation Tools

Creating tools to validate synthetic datasets:

```python
import numpy as np
import matplotlib.pyplot as plt

class DatasetValidator:
    @staticmethod
    def validate_lidar_dataset(file_path):
        """Validate a LiDAR dataset"""
        data = np.load(file_path)
        points = data['points']
        labels = data['labels'] if 'labels' in data.files else None
        
        print(f"Lidar Dataset Validation:")
        print(f"  - Number of points: {len(points)}")
        print(f"  - Point dimensions: {points.shape[1]}")
        print(f"  - X range: [{points[:, 0].min():.2f}, {points[:, 0].max():.2f}]")
        print(f"  - Y range: [{points[:, 1].min():.2f}, {points[:, 1].max():.2f}]")
        print(f"  - Z range: [{points[:, 2].min():.2f}, {points[:, 2].max():.2f}]")
        
        if labels is not None:
            unique_labels, counts = np.unique(labels, return_counts=True)
            print(f"  - Labels: {dict(zip(unique_labels, counts))}")
        
        # Check for NaN or infinite values
        nan_count = np.sum(np.isnan(points))
        inf_count = np.sum(np.isinf(points))
        print(f"  - NaN values: {nan_count}")
        print(f"  - Infinite values: {inf_count}")
        
        return nan_count == 0 and inf_count == 0
    
    @staticmethod
    def validate_camera_dataset(image_path, annotation_path=None):
        """Validate a camera dataset"""
        import cv2
        
        img = cv2.imread(image_path)
        if img is None:
            print(f"Error: Could not load image {image_path}")
            return False
        
        print(f"Camera Dataset Validation:")
        print(f"  - Image shape: {img.shape}")
        print(f"  - Data type: {img.dtype}")
        print(f"  - Value range: [{img.min()}, {img.max()}]")
        
        # Check for valid pixel values
        if img.dtype == np.uint8:
            valid_range = (img >= 0).all() and (img <= 255).all()
        else:
            valid_range = (img >= 0).all() and (img <= 1.0).all()
        
        print(f"  - Valid pixel range: {valid_range}")
        
        return valid_range
    
    @staticmethod
    def validate_imu_dataset(file_path):
        """Validate an IMU dataset"""
        data = np.load(file_path)
        
        print(f"IMU Dataset Validation:")
        
        for key in ['time', 'noisy_acceleration', 'noisy_angular_velocity']:
            if key in data:
                arr = data[key]
                print(f"  - {key}: shape {arr.shape}, range [{arr.min():.4f}, {arr.max():.4f}]")
        
        # Check for valid IMU data characteristics
        acc_magnitude = np.linalg.norm(data['noisy_acceleration'], axis=1)
        gyro_magnitude = np.linalg.norm(data['noisy_angular_velocity'], axis=1)
        
        print(f"  - Acc magnitude range: [{acc_magnitude.min():.4f}, {acc_magnitude.max():.4f}] m/s²")
        print(f"  - Gyro magnitude range: [{gyro_magnitude.min():.4f}, {gyro_magnitude.max():.4f}] rad/s")
        
        # Check for reasonable ranges
        reasonable_acc = (acc_magnitude < 100).all()  # Shouldn't exceed 100 m/s² in normal conditions
        reasonable_gyro = (gyro_magnitude < 100).all()  # Shouldn't exceed 100 rad/s in normal conditions
        
        print(f"  - Reasonable acceleration range: {reasonable_acc}")
        print(f"  - Reasonable angular velocity range: {reasonable_gyro}")
        
        return reasonable_acc and reasonable_gyro

# Example usage of validation
validator = DatasetValidator()

# Validate a few generated files
lidar_valid = validator.validate_lidar_dataset('urban_lidar_dataset.npz')
print(f"Lidar dataset valid: {lidar_valid}\n")

imu_files = [f'imu_trajectory_{i:02d}.npz' for i in range(3)]
for imu_file in imu_files:
    try:
        imu_valid = validator.validate_imu_dataset(imu_file)
        print(f"IMU dataset {imu_file} valid: {imu_valid}\n")
    except FileNotFoundError:
        print(f"IMU file {imu_file} not found\n")
```

These examples demonstrate various approaches to generating synthetic datasets for different types of robotic sensors. The examples include:

1. LiDAR dataset generation for object detection in urban and indoor environments
2. Camera dataset generation for object recognition with various backgrounds
3. Depth dataset generation for depth estimation tasks
4. IMU dataset generation with realistic noise models
5. Multi-sensor fusion dataset combining LiDAR and camera data
6. Dataset validation tools to ensure quality

Each example includes realistic noise models, proper calibration, and validation techniques to ensure the synthetic data is suitable for training AI perception systems.