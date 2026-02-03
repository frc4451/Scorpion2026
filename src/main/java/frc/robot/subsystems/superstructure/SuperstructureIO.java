// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.AutoLog;

public interface SuperstructureIO {
    @AutoLog
    public static class SuperstructureIOInputs {
        public double feederPositionRad = 0.0;
        public double feederVelocityRedPerSec = 0.0;
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
}