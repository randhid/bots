import open3d as o3d

# Create a green box of size 10cm (0.1m cube)
box = o3d.geometry.TriangleMesh.create_box(width=0.1, height=0.1, depth=0.1)
box.paint_uniform_color([0, 1, 0])  # Green

# Move the box to 50cm below and 40cm in front of the camera
box.translate((0, -0.2, 0.4))  # x, y, z in meters

# Sample the surface to create a point cloud
pcd = box.sample_points_poisson_disk(number_of_points=1000)

# Save the point cloud as a .pcd file
o3d.io.write_point_cloud("green_box_10cm.pcd", pcd)
