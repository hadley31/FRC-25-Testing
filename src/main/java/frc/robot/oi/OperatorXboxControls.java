package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorXboxControls implements OperatorControls {
  private final CommandXboxController m_controller;

  public OperatorXboxControls(int port) {
    m_controller = new CommandXboxController(port);
  }

  @Override
  public Trigger stow() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public Trigger scoreArm45Deg() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public Trigger scoreArm75Deg() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public Trigger quasistaticForward() {
    return m_controller.back().and(m_controller.povUp());
  }

  @Override
  public Trigger quasistaticReverse() {
    return m_controller.back().and(m_controller.povDown());
  }

  @Override
  public Trigger dynamicForward() {
    return m_controller.start().and(m_controller.povUp());
  }

  @Override
  public Trigger dynamicReverse() {
    return m_controller.start().and(m_controller.povDown());
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
