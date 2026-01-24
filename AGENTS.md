# AGENTS.md

## Project Overview

This is the robot code for RainMaker26 FRC 360's 26th robot for the 2026 season, REBUILT. It is a Java-based project using WPILib for FIRST Robotics Competition.

## Tech Stack

- Language: Java 17
- Build tool: Gradle (with GradleRIO plugin)
- Frameworks/Libraries: WPILib, AdvantageKit, CTRE Phoenix, Pathplanner

## Setup Commands

# Prerequisites: Install WPILib (https://docs.wpilib.org/en/stable/docs/getting-started/getting-started-frc-control-system/wpilib-setup.html)

# Build the project

./gradlew build

## Testing

Run tests with: ./gradlew test
Always run tests before committing.

## Formatting

Check formatting with: ./gradlew spotlessCheck
Apply formatting with: ./gradlew spotlessApply

## Do

- Write clear, commented code
- Add docstrings to functions
- Keep files small and focused

## Don't

- Hard-code credentials
- Skip error handling
- Delete existing tests

## Ask First

- Before adding new dependencies
- Before restructuring folders
