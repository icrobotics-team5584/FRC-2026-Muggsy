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
