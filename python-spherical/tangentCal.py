import numpy as np

# # Sphere parameters
# sphere_center = np.array([0, 0, 0])  # Sphere center
# sphere_radius = 20  # Sphere radius

# # Point on the sphere
# point_on_sphere = np.array([0, 0, 20])  # Example point on the sphere

# # Calculate the vector from the center to the point
# vector_to_point = point_on_sphere - sphere_center

# # Normalize the vector to get the normal vector of the tangent plane
# normal_vector = vector_to_point / np.linalg.norm(vector_to_point)

# # Define the vector you want to check (e.g., [0, 1, 0])
# vector_to_check = np.array([3, 6, 0])

# # Calculate the angle between the vector and the normal vector
# angle_rad = np.arccos(np.dot(normal_vector, vector_to_check) / (np.linalg.norm(normal_vector) * np.linalg.norm(vector_to_check)))
# angle_deg = np.degrees(angle_rad)

# print(f"Angle between the vector and the tangent plane: {angle_deg:.2f} degrees")

"""--------------------------------------------------------------------------------------------"""

# Sphere parameters
sphere_center = np.array([0, 0, 0])  # Sphere center
sphere_radius = 20.0  # Sphere radius

# Point on the sphere
point_on_sphere = np.array([1.95063980e+01, 1.21593992e-12, 4.41592996e+00])  # Example point on the sphere

# Calculate the vector from the center to the point
vector_to_point = point_on_sphere - sphere_center

# Normalize the vector to get the normal vector of the tangent plane
normal_vector = vector_to_point / np.linalg.norm(vector_to_point)

# Define the tail and head of the vector to check
tail_of_vector = point_on_sphere  # Example origin
head_of_vector = point_on_sphere + np.array([-5.30465668e-02, 2.46476580e-13, 2.28249451e-01])
  # Example endpoint

# Calculate the vector to check
vector_to_check = head_of_vector - tail_of_vector

# Calculate the angle between the vector and the normal vector
angle_rad = np.arccos(np.dot(normal_vector, vector_to_check) / (np.linalg.norm(normal_vector) * np.linalg.norm(vector_to_check)))
angle_deg = np.degrees(angle_rad)

print(f"Angle between the vector to check and the tangent plane: {angle_deg:.2f} degrees")