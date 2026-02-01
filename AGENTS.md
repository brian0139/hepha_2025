# Repository Guidelines

## Project Structure & Module Organization

- `TeamCode/`: primary robot code (OpModes, hardware, Road Runner). Main sources: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`. Android resources: `TeamCode/src/main/res/`.
- `FtcRobotController/`: FTC SDK controller module (generally avoid editing unless you’re updating the SDK or controller app behavior).
- `MeepMeepTesting/`: desktop Road Runner path preview tooling (Java 11) under `MeepMeepTesting/src/main/java/`.
- `ftc-dashboard/`: local `ftc-dashboard` sources included via composite build in `settings.gradle`. Follow `ftc-dashboard/AGENTS.md` when editing this subtree.
- `doc/`: reference docs and media.

## Build, Test, and Development Commands

Run from repo root:

- `./gradlew assembleDebug`: builds debug APK(s) for the Android modules.
- `./gradlew :TeamCode:assembleDebug`: builds only the robot app module.
- `./gradlew installDebug`: installs the debug build to a connected Android device (requires Android SDK + adb).
- `./gradlew clean`: clears Gradle build outputs.

MeepMeep (recommended): run `com.example.meepmeeptesting.MeepMeepTesting` from your IDE after `./gradlew :MeepMeepTesting:build`.

## Coding Style & Naming Conventions

- Language: Java (Android modules target Java 8; `MeepMeepTesting` targets Java 11).
- Formatting: 4-space indentation; keep imports organized; prefer small, single-purpose methods for OpModes.
- Naming: `PascalCase` classes (e.g., `AutonBluePath`), `camelCase` members, constants in `UPPER_SNAKE_CASE`.
- File hygiene: don’t commit local/OS artifacts (e.g., `.DS_Store`) or machine-specific Gradle/IDE files.

## Testing Guidelines

- No dedicated unit-test suite is enforced for `TeamCode`. Validate changes with:
  - MeepMeep previews for trajectories (`MeepMeepTesting/`), and
  - on-robot runs (confirm OpMode selection, telemetry, and Logcat for errors).

## Commit & Pull Request Guidelines

- Current history favors short, informal subjects (e.g., `auton update`, `save commit`). For new work, use a clear imperative summary: `Fix flywheel idle speed` or `Tune RR constraints`.
- PRs should include: what changed, how it was tested (MeepMeep/on-robot), and any driver-facing control changes (brief mapping notes). Avoid committing `local.properties` or secrets.

