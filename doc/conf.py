# Configuration file for the Sphinx documentation builder.
import os, sys
from datetime import datetime
sys.path.insert(0, os.path.abspath(".."))
# Project information
project = "Real-Sim"
author = "Real-Twin Team " 
release = "0.1"  # version string
root_doc = "index" # ensure docs/index.md or docs/index.rst exists

#General configuration
extensions = [
    "myst_parser",          # enable Markdown (MyST)
    "sphinx.ext.autodoc",   # pull in docstrings
    "sphinx.ext.napoleon",  # Google/NumPy style docstrings
     "sphinx.ext.viewcode"    
]

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

#Options for HTML output
html_theme_options = {
    "collapse_navigation": False,
    "sticky_navigation": True,
    "includehidden": True,        
    "navigation_depth": 4,        # <-- ensure depth is enough for your nesting
}

