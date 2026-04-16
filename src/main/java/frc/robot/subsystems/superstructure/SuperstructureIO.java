// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface SuperstructureIO {
  @AutoLog
  public static class SuperstructureIOInputs {
    public double feederPositionRad = 0.0;
    public double feederVelocityRadPerSec = 0.0;
    public double feederAppliedVolts = 0.0;
    public double feederCurrentAmps = 0.0;

    public double agitatorPositionRad = 0.0;
    public double agitatorVelocityRPM = 0.0;
    public double agitatorAppliedVolts = 0.0;
    public double agitatorCurrentAmps = 0.0;

    public double intakeLauncherPositionRad = 0.0;
    public double intakeLauncherVelocityRPM = 0.0;
    public double intakeLauncherAppliedVolts = 0.0;
    public double intakeLauncherCurrentAmps = 0.0;
  }

  // Update the set of loggable inputs.
  public default void updateInputs(SuperstructureIOInputs inputs) {}

  // Run the feeder at the specified voltage.
  public default void setFeederVoltage(double volts) {}

  // Run the intake and launcher at the specified voltage.
  public default void setIntakeLauncherVoltage(double volts) {}

  // For shooter velocity
  public default void setIntakeLauncherVelocity(AngularVelocity shootingVelocity) {}

  // For agitator velocity
  public default void setAgitatorVoltage(double volts) {}
}
