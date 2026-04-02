package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.FieldConstants;
import frc.robot.bobot_state.BobotState;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;

public class LaunchCalculator {
  private static final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();

  static LoggedTunableNumber manualFeetFromHub =
      new LoggedTunableNumber("Interpolation Test/Manual Feet From Hub", 6.0);

  static {
    distanceToRPM.put(Units.feetToMeters(5), -2800.0);
    distanceToRPM.put(Units.feetToMeters(6), -2900.0);
    distanceToRPM.put(Units.feetToMeters(7), -3085.0);
    distanceToRPM.put(Units.feetToMeters(8), -3200.0);
    distanceToRPM.put(Units.feetToMeters(9), -3400.0);
    distanceToRPM.put(Units.feetToMeters(10), -3630.0);
    distanceToRPM.put(Units.feetToMeters(11), -3700.0);
    distanceToRPM.put(Units.feetToMeters(12), -3850.0);
  }

  public static AngularVelocity getVelocityFromDistance(Distance distance) {
    return RPM.of(distanceToRPM.get(distance.in(Meters)));
  }

  public static Pose2d robotPose2d = BobotState.getGlobalPose();
  public static Translation2d hubTranslation2d = AllianceFlipUtil.apply(FieldConstants.Hub.centerOfHub.toTranslation2d());
  public static double distanceToHubMeters = hubTranslation2d.getDistance(robotPose2d.getTranslation());
}
