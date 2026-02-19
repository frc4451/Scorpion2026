package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yaw = new Rotation2d();
  }

  public default void updateInputs(GyroIOInputsAutoLogged gyroIOInputs) {}
}
