"""Point Markdown links that leave doc/ at the file on GitHub.

The pages are also read on GitHub, so they link repo files relatively
(``../tests/Vissim/Ipg/``, ``../CLAUDE.md``). Sphinx only builds doc/, so on
Read the Docs those links have no target. The same goes for a link to a page
kept out of the site by ``exclude_patterns`` (internal design documents). This
rewrites each one whose target exists in the repo to a GitHub URL pinned to the
commit being built, so every docs version links the code it was built from. A link whose target does not
exist anywhere is left alone, so MyST still warns about it.
"""
import functools
import os
import subprocess
from urllib.parse import unquote

from docutils import nodes
from sphinx import addnodes
from sphinx.transforms.post_transforms import SphinxPostTransform


@functools.lru_cache(maxsize=None)
def _commit(repo_root):
    sha = os.environ.get("READTHEDOCS_GIT_COMMIT_HASH")
    if sha:
        return sha
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=repo_root, text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "main"  # local build outside a usable checkout


class RepoLinks(SphinxPostTransform):
    default_priority = 8  # MyST's resolver runs at 9 and warns on what's left

    def run(self, **kwargs):
        srcdir = os.path.abspath(self.env.srcdir)
        repo_root = os.path.dirname(srcdir)
        base_url = "{}/{{kind}}/{}/{{path}}".format(
            self.config.repo_links_github.rstrip("/"), _commit(repo_root)
        )
        for node in list(self.document.findall(addnodes.pending_xref)):
            if node.get("reftype") != "myst":
                continue
            if node["refdomain"] == "doc":
                # MyST already made these relative to doc/ and dropped ".md".
                # A built page resolves normally; a .md outside doc/ or one
                # left out of the site (exclude_patterns) goes to GitHub.
                if node["reftarget"] in self.env.all_docs:
                    continue
                target, anchor = node["reftarget"] + ".md", node.get("reftargetid")
                path = os.path.join(srcdir, target)
            else:
                target, _, anchor = unquote(node["reftarget"]).partition("#")
                refdoc = self.env.doc2path(node.get("refdoc", self.env.docname))
                path = os.path.join(os.path.dirname(refdoc), target)
                if not os.path.relpath(path, srcdir).startswith(".."):
                    continue  # inside doc/: Sphinx/MyST handle it
            path = os.path.normpath(path)
            rel = os.path.relpath(path, repo_root)
            if rel.startswith("..") or not os.path.exists(path):
                continue
            url = base_url.format(
                kind="tree" if os.path.isdir(path) else "blob",
                path=rel.replace(os.sep, "/"),
            )
            if anchor:
                url += "#" + anchor
            ref = nodes.reference("", "", refuri=url, internal=False)
            ref.extend(child.deepcopy() for child in node[0].children)
            if not ref.children:
                ref.append(nodes.literal(target, target))
            node.replace_self(ref)


def setup(app):
    app.add_config_value("repo_links_github", "", "env")
    app.add_post_transform(RepoLinks)
    return {"parallel_read_safe": True, "parallel_write_safe": True}
