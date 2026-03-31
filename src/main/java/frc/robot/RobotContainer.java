// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.bobot_state.BobotState;
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
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final CommandCustomXboxController driveController =
      new CommandCustomXboxController(ControllerConstants.kDriverControllerPort);
  private final CommandCustomXboxController opController =
      new CommandCustomXboxController(ControllerConstants.kOperaterControllerPort);

  private final DriveSubsystem driveSubsystem;
  private final Superstructure superstructure;

  private final LoggedDashboardChooser<Command> autoChooser;

  private final Vision vision = new Vision();

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
        new LoggedDashboardChooser<Command>("Auto Choices", AutoBuilder.buildAutoChooser());
    configureAutos();

    NamedCommands.registerCommand("Launch", superstructure.autoLaunch(2.5));
    NamedCommands.registerCommand("Eject", superstructure.eject());
    NamedCommands.registerCommand("Intake", superstructure.intake());
  }

  private void configureAutos() {
    Command SimpleAuto = AutoBuilder.buildAuto("Simple");
    Command PokemonAuto = AutoBuilder.buildAuto("Pokemon");

    autoChooser.addOption(
        "Sit and Shoot",
        Commands.sequence(Commands.deadline(Commands.waitSeconds(10), superstructure.launch())));
    autoChooser.addOption(
        "Feed Forward Characterization", driveSubsystem.feedforwardCharacterization());
    autoChooser.addOption("Simple", SimpleAuto);
    autoChooser.addOption("Pokemon", PokemonAuto);
  }

  // public Command getAutonomousPath() {
  //   PathPlannerPath path = PathPlannerPath.fromPathFile("Backwards");
  //   return new SequentialCommandGroup(
  //       new InstantCommand(() -> driveSubsystem.setPose(path.getStartingHolonomicPose())));
  //   AutoBuilder.followPath(path);
  // }

  private void configureBindings() {
    driveSubsystem.setDefaultCommand(
        driveSubsystem.driveCommand(
            () -> -driveController.getLeftY(), () -> -driveController.getRightX())
        // driveSubsystem.setDrivetrainArcadeDrive(
        //     () -> -controller.getLeftY(), () -> -controller.getRightX())
        );

    driveController.rightTrigger().whileTrue(superstructure.intake());
    opController.leftTrigger().whileTrue(superstructure.eject());
    opController.rightTrigger().whileTrue(superstructure.launch());

    opController.a().whileTrue(superstructure.setRPSLauncherCommand(RPM.of(2800)));

    driveController
        .x()
        .whileTrue(
            driveSubsystem.driveWithExactHeading(
                () -> {
                  Translation2d targetTranslation =
                      FieldConstants.Hub.centerOfHub.toTranslation2d();
                  Translation2d currentTranslation = BobotState.getGlobalPose().getTranslation();
                  return new Rotation2d(
                      targetTranslation.getX() - currentTranslation.getX(),
                      targetTranslation.getY() - currentTranslation.getY());
                },
                () -> -driveController.getLeftY()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
// EVERYTHING COMMENTED IS WHAT WE ATTEMPTED TO DO - EXCLUDING LINE 42
