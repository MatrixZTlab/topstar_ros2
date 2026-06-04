from __future__ import annotations

from topstar_h1.backends.base import H1Backend


def create_backend(
    backend: str,
    *,
    sim_path: str,
    upper_body_config: dict | None,
    use_mock_upper_body: bool,
    frequency: int,
) -> H1Backend:
    if backend == "mujoco":
        from topstar_h1.backends.mujoco import H1MujocoBackend

        return H1MujocoBackend(
            sim_path=sim_path,
            upper_body_config=upper_body_config,
            use_mock_upper_body=use_mock_upper_body,
            frequency=frequency,
        )
    if backend == "isaac":
        from topstar_h1.backends.isaac import H1IsaacBackend

        return H1IsaacBackend()
    if backend == "xapi":
        from topstar_h1.backends.xapi import H1XapiBackend

        return H1XapiBackend(
            upper_body_config=upper_body_config,
            frequency=frequency,
        )
    raise ValueError(f"Unsupported H1 backend '{backend}'")