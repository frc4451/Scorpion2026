
package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.AutoLog;

public interface SuperstructureIO extends SubsystemBase {
    public double feederPositionRad = 0.0;
    public double feederVelocityRadPerSec = 0.0;
    public double feederAppliedVolts = 0.0;
    public double feederCurrentAmps = 0.0;

    public double intakeLauncherPositionRad = 0.0;
    public double intakeLauncherVelocityRadPerSec = 0.0;
    public double intakeLauncherAppliedVolts = 0.0;
    public double intakeLauncherCurrentAmps = 0.0;
}

public default void updateInputs(SuperstructureIOInputs inputs) {}

public default void setFeederVoltage(double volts) {}

public default void setIntakeLauncherVoltage(double volts) {}