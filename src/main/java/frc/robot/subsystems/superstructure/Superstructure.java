// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.intakingFeederVoltage;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.intakingIntakeVoltage;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.launchingAgitatorVoltage;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.launchingFeederVoltage;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.spinUpFeederVoltage;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private final SuperstructureIO io;
  private final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged();

  private final LoggedTunableNumber tunableRPM =
      new LoggedTunableNumber(getName() + "/IntakeLauncherRPM", 3000.0);

  public Superstructure(SuperstructureIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }

  /** Set the rollers to the values for intaking. */
  public Command intake() {
    return runEnd(
        () -> {
          io.setFeederVoltage(intakingFeederVoltage);
          io.setIntakeLauncherVoltage(intakingIntakeVoltage);
        },
        () -> {
          io.setFeederVoltage(0.0);
          io.setIntakeLauncherVoltage(0.0);
        });
  }

  /** Set the rollers to the values for ejecting fuel out the intake. */
  public Command eject() {
    return runEnd(
        () -> {
          io.setFeederVoltage(-intakingFeederVoltage);
          io.setIntakeLauncherVoltage(-intakingIntakeVoltage);
        },
        () -> {
          io.setFeederVoltage(0.0);
          io.setIntakeLauncherVoltage(0.0);
        });
  }

  /** Set the rollers to the values for launching. Spins up before feeding fuel. */
  public Command launch() {
    return run(() -> {
          Logger.recordOutput(getName() + "/LaunchState", "SPIN UP");
          AngularVelocity shootingVelocity =
              Constants.tuningMode ? RPM.of(tunableRPM.get()) : LaunchCalculator.getVelocityToHub();

          // Get the Robot Pose and the Hub Location
          io.setFeederVoltage(spinUpFeederVoltage);
          io.setAgitatorVoltage(launchingAgitatorVoltage);
          io.setIntakeLauncherVelocity(shootingVelocity);

          // io.setAngularVelocity(-spinUpSeconds);
          Logger.recordOutput(getName() + "/Goal", shootingVelocity);
        })
        .until(
            () -> {
              AngularVelocity shootingVelocity =
                  Constants.tuningMode
                      ? RPM.of(tunableRPM.get())
                      : LaunchCalculator.getVelocityToHub();
              return RPM.of(inputs.intakeLauncherVelocityRPM).isNear(shootingVelocity, 0.1);
            })
        // .withTimeout(spinUpSeconds)
        .andThen(
            run(
                () -> {
                  Logger.recordOutput(getName() + "/LaunchState", "FIRING");

                  AngularVelocity shootingVelocity =
                      Constants.tuningMode
                          ? RPM.of(tunableRPM.get())
                          : LaunchCalculator.getVelocityToHub();

                  io.setFeederVoltage(launchingFeederVoltage);
                  io.setAgitatorVoltage(launchingAgitatorVoltage);
                  io.setIntakeLauncherVelocity(shootingVelocity);
                  // io.setIntakeLauncherVoltage(launchingLauncherVoltage);
                }))
        .finallyDo(
            () -> {
              Logger.recordOutput(getName() + "/LaunchState", "IDLE");

              io.setFeederVoltage(0.0);
              io.setAgitatorVoltage(0.0);
              io.setIntakeLauncherVoltage(0.0);
            });
  }

  public Command autoLaunch(double time) {
    return Commands.deadline(Commands.waitSeconds(time), launch());
  }

  public Command setRPSLauncherCommand(AngularVelocity velocity) {
    return run(
        () -> {
          io.setIntakeLauncherVelocity(RPM.of(tunableRPM.get()));
          Logger.recordOutput(getName() + "/SetShooterVelocity", RPM.of(tunableRPM.get()));
        });
  }
}
