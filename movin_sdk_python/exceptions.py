"""Exceptions raised by the public MOVIN SDK API."""


class MissingOptionalDependencyError(ImportError):
    """An optional SDK feature was used without its dependencies installed."""

    def __init__(self, feature: str, extra: str, dependency: str | None = None):
        self.feature = feature
        self.extra = extra
        self.dependency = dependency

        dependency_hint = f" (missing: {dependency})" if dependency else ""
        super().__init__(
            f"{feature} is an optional MOVIN SDK feature{dependency_hint}. "
            f'Install it with: pip install "movin_sdk_python[{extra}]"'
        )
