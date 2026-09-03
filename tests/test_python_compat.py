"""Guard the declared ``requires-python = ">=3.8"`` floor.

Python 3.8 cannot evaluate ``X | None`` (PEP 604, 3.10+) or ``list[int]``
(PEP 585, 3.9+) annotations at import time unless the module postpones them
with ``from __future__ import annotations``.  A single such annotation in a
core module breaks ``import movin_sdk_python`` on older interpreters.
"""

import ast
import pathlib
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1] / "movin_sdk_python"
BUILTIN_GENERICS = {"dict", "frozenset", "list", "set", "tuple", "type"}
MIN_FEATURE_VERSION = (3, 8)


def _module_sources():
    for path in sorted(PACKAGE_ROOT.rglob("*.py")):
        yield path, path.read_text(encoding="utf-8")


def _has_future_annotations(tree):
    for node in tree.body:
        if isinstance(node, ast.ImportFrom) and node.module == "__future__":
            if any(alias.name == "annotations" for alias in node.names):
                return True
    return False


def _runtime_annotations(tree):
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            args = node.args
            for arg in (
                *args.posonlyargs,
                *args.args,
                *args.kwonlyargs,
                *filter(None, (args.vararg, args.kwarg)),
            ):
                if arg.annotation is not None:
                    yield node.lineno, arg.annotation
            if node.returns is not None:
                yield node.lineno, node.returns
        elif isinstance(node, ast.AnnAssign):
            yield node.lineno, node.annotation


def _needs_newer_python(annotation):
    for sub in ast.walk(annotation):
        if isinstance(sub, ast.BinOp) and isinstance(sub.op, ast.BitOr):
            return "PEP 604 union"
        if (
            isinstance(sub, ast.Subscript)
            and isinstance(sub.value, ast.Name)
            and sub.value.id in BUILTIN_GENERICS
        ):
            return "PEP 585 builtin generic"
    return None


class Python38CompatibilityTests(unittest.TestCase):
    def test_sources_parse_with_python38_grammar(self):
        for path, source in _module_sources():
            with self.subTest(module=str(path.relative_to(PACKAGE_ROOT))):
                ast.parse(source, filename=str(path), feature_version=MIN_FEATURE_VERSION)

    def test_runtime_annotations_do_not_require_newer_python(self):
        problems = []
        for path, source in _module_sources():
            tree = ast.parse(source, filename=str(path))
            if _has_future_annotations(tree):
                continue
            for lineno, annotation in _runtime_annotations(tree):
                reason = _needs_newer_python(annotation)
                if reason:
                    problems.append(
                        f"{path.relative_to(PACKAGE_ROOT)}:{lineno}: {reason} "
                        f"evaluated at import time; add "
                        f"'from __future__ import annotations'"
                    )
        self.assertEqual(problems, [], "\n".join(problems))


if __name__ == "__main__":
    unittest.main()
