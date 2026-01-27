This folder contains internal-only interfaces for process-local flags and fusion.

- degeneracy_flag.h: Declaration of internal interfaces (degeneracy flag, latest localization, external odom toggle and pose).
- degeneracy_flag.cc: Process-local storage and implementations.

Guidelines:
- Include degeneracy_flag.h only from core/implementation files within cartographer library.
- Do not install or export this header publicly.
- External layers (e.g., cartographer_ros) should use minimal forward declarations instead of including this header.
