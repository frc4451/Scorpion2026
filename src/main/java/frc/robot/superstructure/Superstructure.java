
package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase{
    public final SuperstructureIO io;
    public final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged

    public Superstructure(SuperstructureIO io) {
        this.io = io
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Superstructure", inputs);
    }

    public Command intake() {
        return runEnd(
            () -> {
                io.setFeederVoltage(intakingFeederVoltage);
                io.setIntakeLauncherVoltage(intakingFeederVoltage);
            },
            () -> {
                io.setFeederVoltage(0.0);
                io.setIntakeLauncherVoltage(0.0);
            }
        );
    }

    public Command eject() {
        return runEnd(
            () -> {
                io.setFeederVoltage(-intakingFeederVoltage);
                io.setIntakeLauncherVoltage(-intakingFeederVoltage);
            },
            () -> {
                io.setFeederVoltage(0.0);
                io.setIntakeLauncherVoltage(0.0);
            }
        );
    }

    public Command launch() {
        return run(() -> {
            io.setFeederVoltage(spinUpFeederVoltage);
            io.setIntakeLauncherVoltage(launchingLauncherVoltage);
        })
        .withTimeout(spinUpSeconds)
        .andThen(
            run(
                () -> {
                    io.setFeederVoltage(launchingFeederVoltage);
                    io.setIntakeLauncherVoltage(launchingLauncherVoltage);
                }
            )
        );
        .finallyDo(
            () -> {
                io.setFeederVoltage(0.0);
                io.setIntakeLauncherVoltage(0.0);
            }
        );
    }
}