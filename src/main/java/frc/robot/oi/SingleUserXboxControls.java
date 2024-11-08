package frc.robot.oi;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SingleUserXboxControls implements DriverControls {
  private final CommandXboxController m_controller;

  public SingleUserXboxControls(int port) {
    m_controller = new CommandXboxController(port);
  }

  @Override
  public double driveForwardInput() {
    return MathUtil.applyDeadband(-m_controller.getLeftY(), 0.1);
  }

  @Override
  public double driveLeftInput() {
    return MathUtil.applyDeadband(-m_controller.getLeftX(), 0.1);
  }

  @Override
  public double driveRotateInput() {
    return MathUtil.applyDeadband(-m_controller.getRightX(), 0.1);
  }

  public double driverRightX() {
    return m_controller.getRightX();
  }

  public double driverRightY() {
    return -m_controller.getRightY();
  }

  @Override
  public Trigger robotRelativeDrive() {
    return m_controller.rightBumper();
  }

  @Override
  public Trigger seedFieldRelative() {
    return m_controller.y();
  }
}
