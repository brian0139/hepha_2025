# Repository Guidelines

## Project Structure & Module Organization

- `TeamCode/`: team-owned robot code (the main Android app module). Most edits belong here.
  - Java: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/...`
  - Resources/assets: `TeamCode/src/main/res/` (e.g., `xml/`, `values/`, `raw/`)
- `FtcRobotController/`: FTC SDK controller library module. Avoid editing unless you’re intentionally updating SDK-level behavior.
- `MeepMeepTesting/`: desktop MeepMeep + Road Runner visualization (plain Java module).
- Build logic: root `build.gradle`, shared Gradle snippets `build.common.gradle` / `build.dependencies.gradle`.

## Build, Test, and Development Commands

Run from repo root:

- `./gradlew :TeamCode:assembleDebug` — builds the debug APK.
- `./gradlew :TeamCode:installDebug` — installs to a connected Android device (Robot Controller phone).
- `./gradlew :TeamCode:lintDebug` — runs Android Lint on the app.
- `./gradlew :MeepMeepTesting:build` — compiles the MeepMeep desktop module.

Tip: MeepMeep is typically run from your IDE by running
`MeepMeepTesting/src/main/java/com/example/meepmeeptesting/MeepMeepTesting.java`.

## Coding Style & Naming Conventions

- Language: Java (Android + FTC SDK). Use 4-space indentation; avoid tabs.
- Naming: `PascalCase` for classes, `camelCase` for methods/fields, `UPPER_SNAKE_CASE` for constants.
- Prefer adding new code under a dedicated package (example: `.../Stanley/`, `.../Aaron/`) rather than the root `teamcode` package.
- OpModes: keep file/class names descriptive; set a clear `@TeleOp(name="...")` / `@Autonomous(name="...")`.

## Testing Guidelines

- No automated unit/instrumentation tests are currently checked in.
- Verify changes by building (`assembleDebug`) and testing on-robot; capture logs when debugging:
  - Robot logs are available at `http://192.168.43.1:8080/logs`.
- Use `MeepMeepTesting/` for trajectory/pose sanity checks before field testing.

## Commit & Pull Request Guidelines

- Existing history uses short, informal summaries (often imperative; sometimes “save commit”). Keep commits small and describable.
  - Suggested pattern: `area: short action` (e.g., `teleop: tune turret PID`, `auton: adjust shoot timing`).
- PRs: include a brief description, how you tested (device/OpMode/MeepMeep), and screenshots/videos when behavior is visual. Follow `.github/PULL_REQUEST_TEMPLATE.md` when present.

## Configuration & Security Notes

- Don’t commit secrets/keys. Release signing can be driven via env vars (`APK_SIGNING_*`) in `build.common.gradle`.
- Avoid editing shared Gradle files unless necessary; prefer module-local changes in `TeamCode/build.gradle`.

