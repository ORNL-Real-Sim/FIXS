## Helper functions for GUI
from pathlib import Path
def enumerate_sumocfg(input_dir: str, recursive: bool = False) -> list[str]:
    """
    Return a sorted list of absolute paths to .sumocfg files found under `input_dir`.

    Behavior:
    - If `input_dir` points to an existing file and has a .sumocfg suffix, return that file.
    - If `input_dir` points to a directory, list files matching `*.sumocfg`.
      If `recursive` is True, search subdirectories as well.
    - Raises FileNotFoundError if the path does not exist.
    """
    p = Path(input_dir)

    # If a file was passed in directly
    if p.exists() and p.is_file():
        if p.suffix.lower() == ".sumocfg":
            return [str(p.resolve())]
        # it's a file but not .sumocfg -> treat as parent dir below

    # Must be a directory for listing
    if not p.exists():
        raise FileNotFoundError(f"Input path does not exist: {input_dir}")

    if not p.is_dir():
        # if it wasn't a .sumocfg file above and is not a directory, return empty
        return []

    if recursive:
        candidates = p.rglob("*.sumocfg")
    else:
        candidates = p.glob("*.sumocfg")

    files = [str(f.resolve()) for f in candidates if f.is_file() and f.suffix.lower() == ".sumocfg"]
    files.sort()
    return files

def enumerate_ipgtestruns(ipg_dir: str, recursive: bool = False) -> list[str]:
    """
    Enumerate candidate IPG testruns in `ipg_dir`.

    Returns a sorted list of absolute paths. The function is forgiving and will
    return either files or directories that look like testruns.

    Heuristics used:
    - If `ipg_dir` is an existing file, return that file.
    - If `ipg_dir` is a directory, return (sorted):
        - any subdirectory (first-class candidate), and
        - any file matching common testrun filename extensions (case-insensitive):
          .txt, .testrun, .tr
      Use `recursive=True` to search subdirectories as well.

    Raises FileNotFoundError if the given path doesn't exist.
    """
    p = Path(ipg_dir)

    # If a file was provided directly
    if p.exists() and p.is_file():
        return [str(p.resolve())]

    if not p.exists():
        raise FileNotFoundError(f"IPG testrun path does not exist: {ipg_dir}")

    if not p.is_dir():
        return []

    # Collect directories first (these often represent named testruns)
    results: list[str] = []
    if recursive:
        dir_iter = (d for d in p.rglob("*") if d.is_dir())
    else:
        dir_iter = (d for d in p.iterdir() if d.is_dir())

    for d in dir_iter:
        results.append(str(d.resolve()))

    # Common file extensions used for testrun definitions
    exts = (".txt", ".testrun", ".tr")
    if recursive:
        file_iter = p.rglob("*")
    else:
        file_iter = p.iterdir()

    for f in file_iter:
        if f.is_file() and f.suffix.lower() in exts:
            results.append(str(f.resolve()))

    # Deduplicate and sort for deterministic order
    unique = sorted(dict.fromkeys(results))
    return unique