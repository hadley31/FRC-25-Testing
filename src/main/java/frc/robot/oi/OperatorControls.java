package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperatorControls {
  public Trigger stow();

  public Trigger scoreArm45Deg();

  public Trigger scoreArm75Deg();

  public Trigger quasistaticForward();

  public Trigger quasistaticReverse();

  public Trigger dynamicForward();

  public Trigger dynamicReverse();

  public Trigger ampScoreRev();

  public Trigger speakerRev();

  public Trigger shoot();
}
