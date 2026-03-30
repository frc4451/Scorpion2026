package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;

public class FieldUtil {
  public static boolean isPoseInsideField(Pose2d target) {
    return target.getX() > 0.0
        && target.getX() < FieldConstants.fieldLength
        && target.getY() > 0.0
        && target.getY() < FieldConstants.fieldWidth;
  }

  public static boolean isPoseInsideField(Pose3d target) {
    return FieldUtil.isPoseInsideField(target.toPose2d());
  }

  public static Alliance getAlliance() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
  }

  public static boolean isBlueAlliance() {
    return FieldUtil.getAlliance() == Alliance.Blue;
  }

  public static boolean isRedAlliance() {
    return FieldUtil.getAlliance() == Alliance.Red;
  }

  public static double getFlipped() {
    return FieldUtil.isRedAlliance() ? -1 : 1;
  }
}
