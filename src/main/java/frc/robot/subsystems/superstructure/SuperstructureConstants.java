// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.superstructure;

public class SuperstructureConstants {
  public static final int feederCanId = 7;
  public static final double feederMotorReduction = 1.0;
  public static final int feederCurrentLimit = 60;

  public static final int intakeLauncherCanId = 8;
  public static final double intakeLauncherMotorReduction = 1.0;
  public static final int intakeLauncherCurrentLimit = 60;

  public static final double kLauncherKp = 0.00025;
  public static final double kLauncherKi = 0.0;
  public static final double kLauncherKd = 0.0;

  public static final double intakingFeederVoltage = 6.0;
  public static final double intakingIntakeVoltage = -4.0;
  public static final double launchingFeederVoltage = -7.0;
  public static final double launchingLauncherVoltage = -7.0;
  public static final double spinUpFeederVoltage = 1.0;
  public static final double spinUpSeconds = 1.8;
}
// 56.5 in from hub
// 26.5 in length
// 26.1-ish in width
