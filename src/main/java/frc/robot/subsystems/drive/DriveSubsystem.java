package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveSubsystem extends SubsystemBase {
  private final DriveIO io;

  private final DriveIOInputsAutoLogged driveIOInputs = new DriveIOInputsAutoLogged();

  public DriveSubsystem(DriveIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(driveIOInputs);
    Logger.processInputs(getName(), driveIOInputs);
  }

  public Command setDrivetrainVoltage() {
    return Commands.run(() -> io.setVoltage(6, 6));
  }

  public Command setDrivetrainOpenLoop(DoubleSupplier leftVolts, DoubleSupplier rightVolts) {
    return run(
        () -> {
          io.setVoltage(leftVolts.getAsDouble(), rightVolts.getAsDouble());
        });
  }

  public Command setDrivetrainArcadeDrive(DoubleSupplier xSupplier, DoubleSupplier zSupplier) {
    return run(
        () -> {
          WheelSpeeds speeds =
              DifferentialDrive.arcadeDriveIK(
                  xSupplier.getAsDouble(), zSupplier.getAsDouble(), true);

          io.setVelocity(
              speeds.left * DriveConstants.kMaxSpeed,
              speeds.right * DriveConstants.kMaxSpeed,
              0,
              0);
        });
  }
}
