// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.controllers.CommandCustomXboxController;
import frc.robot.controllers.ControllerConstants;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOSpark;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon1;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureIO;
import frc.robot.subsystems.superstructure.SuperstructureIOSim;
import frc.robot.subsystems.superstructure.SuperstructureIOSparkNEO;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final CommandCustomXboxController controller =
      new CommandCustomXboxController(ControllerConstants.kDriverControllerPort);

  private final DriveSubsystem driveSubsystem;
  private final Superstructure superstructure;

  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    DriveIO driveIO;
    GyroIO gyroIO;
    SuperstructureIO superstructureIO;

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        driveIO = new DriveIOSpark();
        gyroIO = new GyroIOPigeon1();
        superstructureIO = new SuperstructureIOSparkNEO();
        break;

      case SIM:
        driveIO = new DriveIOSim();
        gyroIO = new GyroIO() {};
        superstructureIO = new SuperstructureIOSim();
        break;

      case REPLAY:
      default:
        driveIO = new DriveIO() {};
        gyroIO = new GyroIO() {};
        superstructureIO = new SuperstructureIO() {};

        break;
    }

    driveSubsystem = new DriveSubsystem(driveIO, gyroIO);
    superstructure = new Superstructure(superstructureIO);
    configureBindings();

    autoChooser =
        new LoggedDashboardChooser<Command>("Auto Choices", new SendableChooser<Command>());
    configureAutos();
  }

  private void configureAutos() {
    autoChooser.addOption(
        "Sit and Shoot",
        Commands.sequence(Commands.deadline(Commands.waitSeconds(5), superstructure.launch())));
    autoChooser.addOption(
        "Feed Forward Characterization", driveSubsystem.feedforwardCharacterization());
    // autoChooser.addOption("Go Forward", driveSubsystem.goFoward());
  }

  private void configureBindings() {
    driveSubsystem.setDefaultCommand(
        driveSubsystem.driveCommand(() -> -controller.getLeftY(), () -> -controller.getRightX())
        // driveSubsystem.setDrivetrainArcadeDrive(
        //     () -> -controller.getLeftY(), () -> -controller.getRightX())
        );

    controller.rightTrigger().whileTrue(superstructure.intake());
    controller.b().whileTrue(superstructure.eject());
    controller.x().whileTrue(superstructure.launch());
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
