# Repository Guidelines

## Project Structure & Module Organization

- `client/`: React + TypeScript dashboard UI (Vite).
- `DashboardCore/`: standalone Java server core (`src/main/java`) plus unit tests (`src/test/java`).
- `FtcDashboard/`: Android library wrapper that packages the built web UI into app assets.
- `FtcRobotController/`, `TeamCode/`: FTC SDK app + example OpModes.
- `config/checkstyle/`: Checkstyle rules.
- `docs/`: published documentation and generated Javadoc.

## Build, Test, and Development Commands

Client (run from `client/`):

- `yarn` (or `npm install`): install dependencies.
- `yarn dev`: start the local dashboard client (defaults to `http://localhost:3000`).
- `yarn build`: TypeScript compile + production build.
- `yarn lint` / `yarn format`: auto-fix ESLint/Prettier issues.

Android/Java (run from repo root; requires Android SDK + Gradle):

- `./gradlew assembleDebug`: build a debug APK.
- `./gradlew :DashboardCore:test`: run JUnit 5 unit tests.
- `./gradlew check`: run verification tasks (includes Checkstyle where configured).

Note: `FtcDashboard`’s Gradle build runs a `client` build and copies `client/dist` into the Android assets.

## Coding Style & Naming Conventions

- Java/Kotlin: 4-space indentation; keep public APIs under `com.acmerobotics.dashboard.*`; follow `config/checkstyle/checkstyle.xml`.
- Client: Prettier + ESLint (see `client/.prettierrc.js` and `client/.eslintrc.json`); prefer `PascalCase` components and `camelCase` variables.

## Testing Guidelines

- Unit tests live in `DashboardCore/src/test/java` (JUnit Jupiter). Add/extend tests for protocol, serialization, and OpMode manager behavior.
- Client changes are primarily validated via linting/typechecking and manual UI verification.

## Commit & Pull Request Guidelines

- Commit subjects are short and imperative, often with an issue/PR suffix (example: `Fix crash when ... (#204)`).
- PRs should include: a clear description of behavior changes, linked issues, screenshots/GIFs for UI updates, and notes on on-robot validation when relevant.

## Security & Configuration Tips

- Frontend server target: set `VITE_REACT_APP_HOST` (e.g., `192.168.49.1` for Android Phone, `192.168.43.1` for Control Hub).
- Avoid committing secrets/keys. Android signing can be supplied via `APK_SIGNING_*` environment variables.
