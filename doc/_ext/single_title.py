"""Warn when a page has more than one top-level heading.

Each top-level heading becomes its own entry in the navigation, so a page with
several ``#`` headings shows up in the left panel as several pages. Under
``-W`` (RTD fail_on_warning, the docs PR check) the warning fails the build.
"""
from docutils import nodes
from sphinx.util import logging

logger = logging.getLogger(__name__)


def _check(app, doctree):
    tops = [n for n in doctree.children if isinstance(n, nodes.section)]
    if len(tops) > 1:
        logger.warning(
            "%d top-level headings (%s); use one '#' title and '##' for sections",
            len(tops), ", ".join(repr(t[0].astext()) for t in tops[1:4]),
            location=(app.env.docname, tops[1].line),
        )


def setup(app):
    app.connect("doctree-read", _check)
    return {"parallel_read_safe": True, "parallel_write_safe": True}
