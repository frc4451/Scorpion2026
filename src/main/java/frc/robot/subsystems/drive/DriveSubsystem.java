package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    private final DriveIO io;

    public DriveSubsystem(DriveIO io) {
        this.io = io;
    }

    public Command setDrivetrainVoltage() {
        return Commands.run(() -> io.setVoltage(6, 6));
    }

}
