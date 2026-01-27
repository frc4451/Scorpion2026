package frc.robot.subsystems.drive;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class DriveSubsystem extends SubsystemBase {
  private final DriveIO driveIO;

  private final GyroIO gyroIO;

  private boolean isBrake = false;

  private final DriveIOInputsAutoLogged driveIOInputs = new DriveIOInputsAutoLogged();
  private final GyroIOInputsAutoLogged gyroIOInputs = new GyroIOInputsAutoLogged();

  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(DriveConstants.kTrackWidthMeters);
  private final double kS =
      Constants.currentMode == Mode.SIM ? DriveConstants.kSimKs : DriveConstants.kMotorKs;
  private final double kV =
      Constants.currentMode == Mode.SIM ? DriveConstants.kSimKv : DriveConstants.kMotorKv;
  private final DifferentialDrivePoseEstimator poseEstimator =
      new DifferentialDrivePoseEstimator(kinematics, new Rotation2d(), 0.0, 0.0, new Pose2d());

  private Rotation2d rawGyroRotation = new Rotation2d();
  private double lastLeftPositionMeters = 0.0;
  private double lastRightPositionMeters = 0.0;

  public DriveSubsystem(DriveIO driveIO, GyroIO gyroIO) {
    this.driveIO = driveIO;
    this.gyroIO = gyroIO;
  }

  @Override
  public void periodic() {
    driveIO.updateInputs(driveIOInputs);
    gyroIO.updateInputs(gyroIOInputs);
    Logger.processInputs(getName(), driveIOInputs);
    Logger.processInputs(getName() + "Gyro", gyroIOInputs);
    if (DriverStation.isDisabled()) {
      if (!isBrake) {
        isBrake = false;
      }
      driveIO.stop();
    } else {
      if (isBrake) {
        isBrake = false;
      }
    }

    if (gyroIOInputs.connected) {
      rawGyroRotation = gyroIOInputs.yaw;
    } else {
      Twist2d twist =
          kinematics.toTwist2d(
              getLeftPositionMeters() - lastLeftPositionMeters,
              getRightPositionMeters() - lastRightPositionMeters);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      lastLeftPositionMeters = getLeftPositionMeters();
      lastRightPositionMeters = getRightPositionMeters();

      poseEstimator.update(rawGyroRotation, getLeftPositionMeters(), getRightPositionMeters());
    }
  }

  @AutoLogOutput
  public double getLeftPositionMeters() {
    return driveIOInputs.leftPositionRad * DriveConstants.kWheelRadiusMeters;
  }

  @AutoLogOutput
  public double getRightPositionMeters() {
    return driveIOInputs.rightPositionRad * DriveConstants.kWheelRadiusMeters;
  }

  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPose(pose);
  }

  @AutoLogOutput
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(
        new DifferentialDriveWheelSpeeds(
            getLeftVelocityMetersPerSecond(), getRightVelocityMetersPerSecond()));
  }

  @AutoLogOutput
  public double getLeftVelocityMetersPerSecond() {
    return driveIOInputs.leftVelocityRadPerSec * DriveConstants.kWheelRadiusMeters;
  }

  @AutoLogOutput
  public double getRightVelocityMetersPerSecond() {
    return driveIOInputs.rightVelocityRadPerSec * DriveConstants.kWheelRadiusMeters;
  }

  public void runClosedLoop(ChassisSpeeds speeds) {
    Logger.recordOutput("DriveSubsystem/SpeedSetpoint", speeds);
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    runClosedLoop(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }

  /**
   * Executes commands given from driveCommand, converts from meters per second to radians per
   * second for velocity
   */
  public void runClosedLoop(double leftMetersPerSec, double rightMetersPerSec) {
    double leftRadPerSec = leftMetersPerSec / DriveConstants.kWheelRadiusMeters;
    double leftRotPerMin = leftRadPerSec * (60 / 2 * Math.PI);
    double rightRadPerSec = rightMetersPerSec / DriveConstants.kWheelRadiusMeters;
    double rightRotPerMin = rightRadPerSec * (60 / 2 * Math.PI);
    Logger.recordOutput("DriveSubsystem/Setpoint/LeftRadPerSec", leftRadPerSec);
    Logger.recordOutput("DriveSubsystem/Setpoint/RightRadPerSec", rightRadPerSec);
    Logger.recordOutput("DriveSubsystem/LeftRPM", leftRotPerMin);
    Logger.recordOutput("DriveSubsystem/RightRPM", rightRotPerMin);

    double leftFFVolts = (kS * Math.signum(leftRadPerSec)) + (kV * leftRadPerSec);
    double rightFFVolts = (kS * Math.signum(rightRadPerSec)) + (kV * rightRadPerSec);

    driveIO.setVelocity(leftRadPerSec, rightRadPerSec, leftFFVolts, rightFFVolts);
  }

  public void drive(double forward, double rotation) {
    WheelSpeeds speeds;
    if (forward != 0) {
      speeds =
          DifferentialDrive.curvatureDriveIK(
              forward * Math.abs(forward), rotation * Math.abs(rotation) / 1.4, false);
    } else {
      speeds = DifferentialDrive.arcadeDriveIK(0, rotation / 2, true);
    }
    Logger.recordOutput("DriveSubsystem/Setpoint/inputtedX", forward);
    runClosedLoop(speeds.left * DriveConstants.kMaxSpeed, speeds.right * DriveConstants.kMaxSpeed);
  }

  public Command driveCommand(DoubleSupplier forward, DoubleSupplier rotation) {
    return Commands.run(() -> drive(forward.getAsDouble(), rotation.getAsDouble()), this);
  }

  public Command setDrivetrainOpenLoop(DoubleSupplier leftVolts, DoubleSupplier rightVolts) {
    return run(
        () -> {
          driveIO.setVoltage(leftVolts.getAsDouble(), rightVolts.getAsDouble());
        });
  }

  public Command setDrivetrainArcadeDrive(DoubleSupplier xSupplier, DoubleSupplier zSupplier) {
    return run(
        () -> {
          WheelSpeeds speeds =
              DifferentialDrive.arcadeDriveIK(
                  xSupplier.getAsDouble(), zSupplier.getAsDouble(), true);

          driveIO.setVelocity(
              speeds.left * DriveConstants.kMaxSpeed,
              speeds.right * DriveConstants.kMaxSpeed,
              0,
              0);
        });
  }
}
