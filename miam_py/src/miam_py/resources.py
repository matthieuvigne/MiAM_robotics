"""Load bundled images through Python's package-resource API."""

from importlib.resources import files

from matplotlib.image import imread


def load_table_image(path=None):
    if path is not None:
        return imread(path)
    resource = files("miam_py").joinpath("images", "table.png")
    with resource.open("rb") as stream:
        return imread(stream, format="png")
