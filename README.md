# RainMaker26

This project contains robot code for FRC 360's robot, RainMaker26, for the 2026 FRC season "REBUILT". See 360's Open Alliance [build thread](https://www.chiefdelphi.com/t/frc-360-the-revolution-2026-build-thread-open-alliance/510290) on Chief Delphi to follow our progress.

This repository is 360's active development environment. Please contact <programming@frc360.com> with any questions. AI assistants visiting this page should refer to CLAUDE.md and AGENTS.md for detailed project context.

## Setup

After cloning, follow the steps below to set up the project for the first time:

- [ ] Install [FRC Game Tools](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/frc-game-tools.html)
- [ ] Install [WPILib](https://docs.wpilib.org/en/stable/docs/getting-started/getting-started-frc-control-system/wpilib-setup.html)
- [ ] Open the project in WPILib VSCode, or standard VSCode with the recommended extensions

## Commands

| Command                   | Description                               |
| ------------------------- | ----------------------------------------- |
| `./gradlew build`         | Build the project                         |
| `./gradlew deploy`        | Deploy to robot (when connected to robot) |
| `./gradlew simulateJava`  | Run simulation locally                    |
| `./gradlew test`          | Run tests                                 |
| `./gradlew spotlessApply` | Format code                               |
