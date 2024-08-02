import freud
import matplotlib.cm
import numpy as np
import plato.draw.fresnel
import rowan

backend = plato.draw.fresnel
# For interactive scenes:
# import plato.draw.pythreejs
# backend = plato.draw.pythreejs
def plot_crystal(
    box,
    positions,
    colors=None,
    radii=None,
    backend=None,
    polytopes=[],
    polytope_colors=None,
):
    if backend is None:
        backend = plato.draw.fresnel
    if colors is None:
        colors = np.array([[0.5, 0.5, 0.5, 1]] * len(positions))
    if radii is None:
        radii = np.array([0.5] * len(positions))
    sphere_prim = backend.Spheres(positions=positions, colors=colors, radii=radii)
    box_prim = backend.Box.from_box(box, width=0.1)
    if polytope_colors is None:
        polytope_colors = colors * np.array([1, 1, 1, 0.4])
    polytope_prims = []
    for p, c in zip(polytopes, polytope_colors):
        p_prim = backend.ConvexPolyhedra(
            positions=[[0, 0, 0]], colors=c, vertices=p, outline=0
        )
        polytope_prims.append(p_prim)
    rotation = rowan.multiply(
        rowan.from_axis_angle([1, 0, 0], np.pi / 10),
        rowan.from_axis_angle([0, 1, 0], -np.pi / 10),
    )
    scene = backend.Scene(
        [sphere_prim, box_prim, *polytope_prims], zoom=3, rotation=rotation
    )
    if backend is not plato.draw.fresnel:
        scene.enable("directional_light")
    else:
        scene.enable("antialiasing")
    scene.show()

np.random.seed(12)
box, positions = freud.data.UnitCell.fcc().generate_system(3, scale=2, sigma_noise=0.05)
cmap = matplotlib.cm.get_cmap("tab20")
colors = cmap(np.random.rand(len(positions)))

plot_crystal(box, positions, colors, backend=backend)
