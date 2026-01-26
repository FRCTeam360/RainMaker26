# RainMaker26

This project contains robot code for FRC 360's robot, RainMaker26, for the 2026 FRC season "REBUILT". See 360's Open Alliance [build thread](https://www.chiefdelphi.com/t/frc-360-the-revolution-2026-build-thread-open-alliance/510290) on Chief Delphi to follow our progress.

This repository is 360's active development environment. Please contact <programming@frc360.com> with any questions.

## Setup

After cloning, follow the steps below to set up the project for the first time:

- [ ] Install the recommended VSCode extensions when prompted
- [ ] Install [WPILib](https://docs.wpilib.org/en/stable/docs/getting-started/getting-started-frc-control-system/wpilib-setup.html)
- [ ] Install [FRC Game Tools](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/frc-game-tools.html)

## Commands

| Command                   | Description                               |
| ------------------------- | ----------------------------------------- |
| `./gradlew build`         | Build the project                         |
| `./gradlew deploy`        | Deploy to robot (when connected to robot) |
| `./gradlew simulateJava`  | Run simulation locally                    |
| `./gradlew test`          | Run tests                                 |
| `./gradlew spotlessApply` | Format code                               |
