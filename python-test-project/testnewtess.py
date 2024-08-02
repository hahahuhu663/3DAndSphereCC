import numpy as np
from tess import Container
# Define the Gaussian density function
def gaussian_density(x, y, z, cx, cy, cz, sigma):
    dx = x - cx
    dy = y - cy
    dz = z - cz
    dist_squared = dx * dx + dy * dy + dz * dz
    coef = 1.0 / (np.sqrt(2.0 * np.pi) * sigma)
    expon = np.exp(-dist_squared / (2.0 * sigma * sigma))
    return coef * expon

# Create a Tess container


# Generate some random input points
np.random.seed(0)
points = np.random.rand(50, 3)

container = Container(points, limits=[(-20,-20,-20),(20,20,20)], periodic=False)


# Compute the density of each Voronoi cell using a Gaussian density function
for cell in container:
    # Get the vertices of the current cell
    vertices = cell.vertices()

    # Compute the centroid of the current cell
    centroid = cell.centroid()
    print(centroid)
    x = np.array([])
    y = np.array([])
    z = np.array([])
    for v in vertices:
        x = np.append(x, np.array(v[0]))
        y = np.append(y, np.array(v[1]))
        z = np.append(z, np.array(v[2]))

    # Compute the density of the current cell using a Gaussian density function
    density = gaussian_density(x, y, z,
                               centroid[0], centroid[1], centroid[2], 1.0)

    print(density)
    # Update the centroid based on the new density
    new_centroid = np.mean(vertices * density[:, np.newaxis], axis=0)

    # Output the new centroid
    print(f"Cell centroid: {new_centroid}")