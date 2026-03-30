// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SuperstructureIOSim implements SuperstructureIO {
  private final DCMotor intakeLauncherMotor = DCMotor.getNEO(1);
  private final DCMotor intakeFeederMotor = DCMotor.getNEO(1);

  private DCMotorSim feederSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(intakeFeederMotor, 0.004, feederMotorReduction),
          intakeFeederMotor);
  private DCMotorSim intakeLauncherSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              intakeLauncherMotor, 0.004, intakeLauncherMotorReduction),
          intakeLauncherMotor);

  private double feederAppliedVolts = 0.0;
  private double intakeLauncherAppliedVolts = 0.0;

  private final PIDController intakeLauncherPID =
      new PIDController(kLauncherKp, kLauncherKi, kLauncherKd);

  private boolean isClosedLoop = false;
  private double intakeLauncherSetpointRPM = 0.0;

  public void updateInputs(SuperstructureIOInputs inputs) {
    feederSim.setInputVoltage(feederAppliedVolts);
    feederSim.update(0.02);

    if (isClosedLoop) {
      double currentRPM = intakeLauncherSim.getAngularVelocityRPM();
      double feedforward = kLauncherKV * intakeLauncherSetpointRPM * 12.0;
      intakeLauncherAppliedVolts =
          MathUtil.clamp(feedforward + intakeLauncherPID.calculate(currentRPM) * 12.0, -12.0, 12.0);
    }
    intakeLauncherSim.setInputVoltage(intakeLauncherAppliedVolts);
    intakeLauncherSim.update(0.02);

    inputs.feederPositionRad = feederSim.getAngularPositionRad();
    inputs.feederVelocityRadPerSec = feederSim.getAngularVelocityRadPerSec();
    inputs.feederAppliedVolts = feederAppliedVolts;
    inputs.feederCurrentAmps = feederSim.getCurrentDrawAmps();

    inputs.intakeLauncherPositionRad = intakeLauncherSim.getAngularPositionRad();
    inputs.intakeLauncherVelocityRPM = intakeLauncherSim.getAngularVelocityRPM();
    inputs.intakeLauncherAppliedVolts = intakeLauncherAppliedVolts;
    inputs.intakeLauncherCurrentAmps = intakeLauncherSim.getCurrentDrawAmps();
  }

  @Override
  public void setFeederVoltage(double volts) {
    feederAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setIntakeLauncherVoltage(double volts) {
    isClosedLoop = false;
    intakeLauncherAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setIntakeLauncherVelocity(AngularVelocity velocity) {
    isClosedLoop = true;
    intakeLauncherSetpointRPM = velocity.in(RPM);
    intakeLauncherPID.setSetpoint(intakeLauncherSetpointRPM);
  }
}
