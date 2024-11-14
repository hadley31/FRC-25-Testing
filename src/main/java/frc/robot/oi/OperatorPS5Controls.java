package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorPS5Controls implements OperatorControls {
  private final CommandPS5Controller m_controller;

  public OperatorPS5Controls(int port) {
    m_controller = new CommandPS5Controller(port);
  }

  @Override
  public Trigger stow() {
    return m_controller.cross();
  }

  @Override
  public Trigger scoreArm45Deg() {
    return m_controller.circle();
  }

  @Override
  public Trigger scoreArm75Deg() {
    return m_controller.triangle();
  }

  @Override
  public Trigger quasistaticForward() {
    return m_controller.create().and(m_controller.povUp());
  }

  @Override
  public Trigger quasistaticReverse() {
    return m_controller.create().and(m_controller.povDown());
  }

  @Override
  public Trigger dynamicForward() {
    return m_controller.options().and(m_controller.povUp());
  }

  @Override
  public Trigger dynamicReverse() {
    return m_controller.options().and(m_controller.povDown());
  }

  @Override
  public Trigger ampScoreRev() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'ampScoreRev'");
  }

  @Override
  public Trigger speakerRev() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'speakerRev'");
  }

  @Override
  public Trigger shoot() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'shoot'");
  }
}
