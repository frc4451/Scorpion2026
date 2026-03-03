// package frc.robot.autonomous;

// import static edu.wpi.first.units.Units.Seconds;

// import edu.wpi.first.units.measure.Time;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.lib.BLine.Path;
// import frc.robot.subsystems.drive.DriveIO;
// import frc.robot.subsystems.drive.DriveSubsystem;
// import frc.robot.subsystems.superstructure.Superstructure;

// public class AutoManager {
//   private final DriveSubsystem driveSubsystem;
//   private final Superstructure superstructure;
//   private final DriveIO driveIO;

//   private static final Time MAX_TIMEOUT = Seconds.of(20);

//   public AutoManager(
//       DriveSubsystem driveSubsystem, Superstructure superstructure, DriveIO driveIO) {
//     this.driveSubsystem = driveSubsystem;
//     this.superstructure = superstructure;
//     this.driveIO = driveIO;
//   }

//   private Command runLaunch(double time) {
//     return Commands.deadline(Commands.waitSeconds(time), superstructure.launch());
//   }

//   private Command runIntake(double time) {
//     return Commands.deadline(Commands.waitSeconds(time), superstructure.intake());
//   }

//   private Command runEject(double time) {
//     return Commands.deadline(Commands.waitSeconds(time), superstructure.eject());
//   }

//   private Command autoStop() {
//     return Commands.deadline(Commands.waitSeconds(0), driveIO.stop());
//   }

//   private Command BLineTrajectory(String name) {
//     return Commands.sequence(driveSubsystem.getBLineBuilder().build(new Path(name)))
//         .beforeStarting(Commands.print(name + ": start"))
//         .andThen(Commands.print(name + ": end"))
//         .andThen(driveIO.stop());
//   }

//   public Command oneFoot() {
//     return Commands.sequence(
//         BLineTrajectory("one_foot")
//             .beforeStarting(Commands.print("Path start"))
//             .andThen(Commands.print("Path end")));
//   }

//   public Command twoFoot() {
//     return Commands.sequence(
//         BLineTrajectory("two_foot")
//             .beforeStarting(Commands.print("Path start"))
//             .andThen(Commands.print("Path end")));
//   }

//   public Command redFoot() {
//     return Commands.sequence(
//         BLineTrajectory("red_foot")
//             .beforeStarting(Commands.print("Path start"))
//             .andThen(Commands.print("Path end")));
//   }

//   public Command blueFoot() {
//     return Commands.sequence(
//         BLineTrajectory("blue_foot")
//             .beforeStarting(Commands.print("Path start"))
//             .andThen(Commands.print("Path end")));
//   }

//   public Command fiveFeetThenShoot() {
//     return Commands.sequence(
//         BLineTrajectory("five_feet"),
//         this.runLaunch(6)
//             .beforeStarting(Commands.print("Path start"))
//             .andThen(Commands.print("Path end")));
//   }
// }
