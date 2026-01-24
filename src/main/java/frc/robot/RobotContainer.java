// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.controllers.CommandCustomXboxController;
import frc.robot.controllers.ControllerConstants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOSpark;
import frc.robot.subsystems.drive.DriveSubsystem;

public class RobotContainer {
  private final CommandCustomXboxController controller =
      new CommandCustomXboxController(ControllerConstants.kDriverControllerPort);

  private final DriveSubsystem driveSubsystem;

  public RobotContainer() {
    DriveIO driveIO;

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        driveIO = new DriveIOSpark();
        break;

      case SIM:
        driveIO = new DriveIOSim();
        break;

      case REPLAY:
      default:
        driveIO = new DriveIO() {};
        break;
    }

    driveSubsystem = new DriveSubsystem(driveIO);

    configureBindings();
  }

  private void configureBindings() {
    driveSubsystem.setDefaultCommand(
        driveSubsystem.setDrivetrainOpenLoop(
            () -> -controller.getLeftY() * DriveConstants.kOpenLoopVoltageScalar, () -> -controller.getRightX()* DriveConstants.kOpenLoopVoltageScalar)
        // driveSubsystem.setDrivetrainArcadeDrive(
        //     () -> -controller.getLeftY(), () -> -controller.getRightX())
        );
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
