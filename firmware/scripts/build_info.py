"""Give the firmware its version, its variant and its dashboard.

The version comes from the VERSION file in the repository root, the variant from the
environment, as in the name of a release file, and the dashboard from
firmware/web/dashboard.html, embedded as a string.
"""

from pathlib import Path

Import("env")  # noqa: F821

project = Path(env.subst("$PROJECT_DIR"))  # noqa: F821

version = (project.parent / "VERSION").read_text(encoding="utf-8").strip()
variant = env["PIOENV"].replace("-", "")  # noqa: F821
env.Append(  # noqa: F821
    CPPDEFINES=[
        ("TADO_GADO_VERSION", env.StringifyMacro(version)),  # noqa: F821
        ("TADO_GADO_VARIANT", env.StringifyMacro(variant)),  # noqa: F821
    ]
)

page = (project / "web" / "dashboard.html").read_text(encoding="utf-8")
if ")TADOGADOHTML" in page:
    raise ValueError("dashboard.html contains the raw string delimiter )TADOGADOHTML")
generated = project / "src" / "generated"
generated.mkdir(exist_ok=True)
header = f'#pragma once\nstatic const char kDashboardHtml[] = R"TADOGADOHTML({page})TADOGADOHTML";\n'
target = generated / "dashboard_html.h"
if not target.exists() or target.read_text(encoding="utf-8") != header:
    target.write_text(header, encoding="utf-8")
