# Configuration file for the Sphinx documentation builder.
import os, sys
from datetime import datetime
sys.path.insert(0, os.path.abspath(".."))
sys.path.insert(0, os.path.abspath("_ext"))
# Project information
project = "Real-Sim"
author = "Real-Sim Team "
# The version RTD is building (v0.9.1-alpha, stable, latest). RTD clones
# shallow and without tags, so git describe can't answer this there.
release = os.environ.get("READTHEDOCS_VERSION_NAME", "local")
root_doc = "index" # ensure doc/index.md or doc/index.rst exists

#General configuration
extensions = [
    "myst_parser",          # enable Markdown (MyST)
    "sphinx.ext.autodoc",   # pull in docstrings
    "sphinx.ext.napoleon",  # Google/NumPy style docstrings
     "sphinx.ext.viewcode",
    "repo_links",           # links out of doc/ -> GitHub at the built commit
]

repo_links_github = "https://github.com/ORNL-Real-Sim/FIXS"

templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

# Recognize both .rst and .md files
source_suffix = {
    ".rst": "restructuredtext",
    ".md": "markdown",
}

#MyST options
myst_enable_extensions = [
    "colon_fence",   # ::: fenced blocks
    "deflist",       # definition lists
    "linkify",       # auto-detect links
]

# Ensure anchors are generated for headings up to H5 so manual Markdown links
# (e.g., `[Section](#section-name)`) remain stable across RTD builds.
myst_heading_anchors = 5

#Options for HTML output
html_theme = "sphinx_rtd_theme"
html_theme_options = {
    "collapse_navigation": False,
    "sticky_navigation": True,
    "includehidden": True,        
    "navigation_depth": 4,        # <-- ensure depth is enough for your nesting
}

