# Repository Guidance

## Project scope

- This repository is a ROS 2 Humble workspace.
- Keep ROS packages under `src/` and workspace-wide helper scripts under `scripts/`.
- Do not add generated files from `build/`, `install/`, or `log/` to Git.
- Preserve unrelated user changes when working in a dirty worktree.

## Build and verification

- Use `./scripts/build.sh` to build the complete workspace.
- For a focused change, build only the affected package first:
  `./scripts/build.sh --packages-select <package_name>`.
- Run checks appropriate to the changed package before committing.
- Report any check that could not be run and explain why.

## Change boundaries

- Keep each change focused on one topic.
- Do not mix robot description, control, planning, and hardware-driver concerns.
- Put module-specific guidance in an `AGENTS.md` inside that package.
- Do not delete, overwrite, or reformat unrelated files.

## Git conventions

- Use commit messages in the format `type(module): Chinese change summary`.
- Common types are `feat`, `fix`, `refactor`, `docs`, `test`, `build`, and `chore`.
- Each commit must contain only one topic.
- Do not commit build products, logs, caches, secrets, or machine-local configuration.
- Do not push commits unless the user explicitly requests it.

## ROS conventions

- Declare package dependencies in `package.xml` and the package build configuration.
- Prefer package-relative resource paths over machine-specific absolute paths.
- Keep launch files, configuration, URDF/Xacro, and source code in their conventional package directories.
- Treat CAD-exported dynamics, limits, and collision geometry as unverified until checked against hardware specifications.
