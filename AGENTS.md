## Building

Use the bundled WPILib JDK; do not hard-code its architecture or use the system JDK.

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

If WPILib is elsewhere, point `JAVA_HOME` at its `jdk` directory. Always use
`./gradlew build`, never `./gradlew.bat build`.
