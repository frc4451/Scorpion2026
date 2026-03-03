package frc.robot.autonomous;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.superstructure.Superstructure;

public class AutoManager {
  private final DriveSubsystem drive;
  private final Superstructure superstructure;

  private static final Time MAX_TIMEOUT = Seconds.of(20);

  public AutoManager(DriveSubsystem drive, Superstructure superstructure) {
    this.drive = drive;
    this.superstructure = superstructure;
  }

  private Command runLaunch(double time) {
    return Commands.deadline(Commands.waitSeconds(time), superstructure.launch());
  }

  private Command runIntake(double time) {
    return Commands.deadline(Commands.waitSeconds(time), superstructure.intake());
  }

  private Command runEject(double time) {
    return Commands.deadline(Commands.waitSeconds(time), superstructure.eject());
  }

  private Command BLineTrajectory(String name) {
    return Commands.sequence(drive.getBLineBuilder().build(new Path(name)))
        .beforeStarting(Commands.print(name + ": start"))
        .andThen(Commands.print(name + ": end"));
  }

  public Command fiveFeetThenShoot() {
    return Commands.sequence(
        BLineTrajectory("five_feet"),
        this.runLaunch(6)
            .beforeStarting(Commands.print("Path start"))
            .andThen(Commands.print("Path end")));
  }
}
