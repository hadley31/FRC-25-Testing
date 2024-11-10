package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorPS5Controls implements OperatorControls {
  private final CommandPS5Controller m_controller;

  public OperatorPS5Controls(int port) {
    m_controller = new CommandPS5Controller(port);
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
}