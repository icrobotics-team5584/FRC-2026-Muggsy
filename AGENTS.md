## Building

Use WPILib's bundled JDK, never the system JDK or an architecture-specific path.
Adjust `JAVA_HOME` if WPILib is installed elsewhere.

```powershell
$env:JAVA_HOME = 'C:\Users\Public\wpilib\2026\jdk'
$env:Path = "$env:JAVA_HOME\bin;$env:Path"
./gradlew build
```

```sh
# macOS or Linux
export JAVA_HOME="$HOME/wpilib/2026/jdk"
export PATH="$JAVA_HOME/bin:$PATH"
./gradlew build
```

After feature or test changes, run `./gradlew build` (never `./gradlew.bat build`).
Read console errors, fix their causes—including unrelated failures—and rebuild
until the full build succeeds, unless the user explicitly asks otherwise.

## Testing

Add automated tests for every feature or behavioural change, covering intended
behaviour, boundaries, and failure cases. Run relevant tests before finishing;
state any remaining hardware-only validation.


## ICRobotics Software Standards 2026

- Simplicity: Keep small, single-use logic inline in the existing function. Do not create a separate utility, class, or file for a simple condition; extract an abstraction only when reuse or meaningful complexity justifies it.
- Git: Use short, clear, logical commit messages; prefix the robot feature when unclear. Create a PascalCase feature branch (e.g. `VisionTargeting`). Before merging, merge latest `main` into it and test all robot features for correct, safe operation. Commit and push the feature branch, then open a PR targeting `main`. Wait for mentor/software team lead approval; do not self-approve.
- Formatting: Keep `.clang-format` at the project root, using WPILib rules with a 100-column limit. Apply Format Document (`Alt+Shift+F`).
- Variables: Use `_camelCase` for class members; omit the leading underscore for locals. Define hard-coded constants as `constexpr` or `const` with `UPPER_SNAKE_CASE` names; prefer `static constexpr` for class constants.
- Ownership: Prefer direct objects. When dynamic creation/delayed initialisation is necessary, use smart pointers: prefer `std::unique_ptr`; use `std::shared_ptr` only when no single deletion owner is clear. Pass large objects by reference. Avoid raw owning pointers and manual `new`/`delete`.
- Functions: Use PascalCase names, preferably verbs; boolean functions may ask questions (`IsFinished`, `HasRunEnded`). Document every non-trivial function above its header declaration, explaining purpose and relevant constraints, recommendations and edge cases.
- Includes: Use `"..."` for project headers and `<...>` for external headers.
- Comments: Always add concise comments to major blocks of code and individual lines where they improve readability. Explain intent, reasoning, or non-obvious behaviour rather than restating the code, and keep comments accurate when changing the implementation.
- Subsystems: Prefix subsystem classes with `Sub` (e.g. `SubIntake`) to distinguish them from commands.
- Hardware constants: Centralise all robot and driver-station component IDs/ports in `Constants.h`, using `UPPER_SNAKE_CASE` and appropriate namespaces (e.g. `canid`, `dio`, `pcm`), so wiring changes and duplicate assignments are easy to identify.
- Structure: Store robot code under `src/main`: headers in `include`, implementations in `cpp`, each organised into `subsystems`, `commands` and `utilities`. Use lowercase folder names and PascalCase filenames.

