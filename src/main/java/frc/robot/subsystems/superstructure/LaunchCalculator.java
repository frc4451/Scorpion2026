package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

public class LaunchCalculator {
    static InterpolatingDoubleTreeMap distanceToRPM;

    static LoggedTunableNumber manualFeetFromHub = new LoggedTunableNumber("Interpolation Test/Manual Feet From Hub", 6.0);

    static
    {
        distanceToRPM.put(Units.feetToMeters(manualFeetFromHub), rpmAtDistance);
    }
}
