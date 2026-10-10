"""Load bundled images through Python's package-resource API."""

from importlib.resources import files

import numpy as np
from matplotlib.image import imread


def load_table_image() -> np.ndarray:
    """Read the packaged table image."""
    resource = files("miam_py").joinpath("images", "table.png")
    with resource.open("rb") as stream:
        return imread(stream, format="png")
