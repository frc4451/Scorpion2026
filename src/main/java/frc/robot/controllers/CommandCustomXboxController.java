package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class CommandCustomXboxController extends CommandXboxController {

  public CommandCustomXboxController(int port) {
    super(port);
  }

  @Override
  public double getLeftX() {
    return MathUtil.applyDeadband(super.getLeftX(), ControllerConstants.kJoystickDeadband);
  }

  public double getLeftXSquare() {
    return Math.abs(getLeftX()) * getLeftX();
  }

  @Override
  public double getLeftY() {
    return MathUtil.applyDeadband(super.getLeftY(), ControllerConstants.kJoystickDeadband);
  }

  public double getLeftYSquare() {
    return Math.abs(getLeftY()) * getLeftY();
  }

  @Override
  public double getRightX() {
    return MathUtil.applyDeadband(super.getRightX(), ControllerConstants.kJoystickDeadband);
  }

  public double getRightXSquare() {
    return Math.abs(getRightX()) * getRightX();
  }

  @Override
  public double getRightY() {
    return MathUtil.applyDeadband(super.getRightY(), ControllerConstants.kJoystickDeadband);
  }

  public double getRightYSquare() {
    return Math.abs(getRightY()) * getRightY();
  }
}
