package frc.robot.subsystems.superstructure;

import static frc.robot.subsystems.superstructure.SuperstructureConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.systen.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SuperstructureIOSim  implements SuperstructureIO {
    private DCMotorSim feederSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getCIM(1), 0.004, feederMotorReduction),
        DCMotor.getCIM(1)
    );
    private DCMotorSim intakeLauncherSim = new DCMotorSim(LinearSystemId.createDCMotorSystem())
}
