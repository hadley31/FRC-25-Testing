package frc.robot.oi;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;

public interface DriverControls {
  public double driveForwardInput();

  public double driveLeftInput();

  public double driveRotateInput();

  public double driverRightX();

  public double driverRightY();

  public Trigger robotRelativeDrive();

  public Trigger seedFieldRelative();

  public default LinearVelocity driveForwardVelocity() {
    return DriveConstants.kMaxLinearSpeed.times(driveForwardInput());
  }

  public default LinearVelocity driveLeftVelocity() {
    return DriveConstants.kMaxLinearSpeed.times(driveLeftInput());
  }

  public default AngularVelocity driveAngularVelocity() {
    return DriveConstants.kMaxAngularSpeed.times(driveRotateInput());
  }

  public default ChassisSpeeds getTargetChassisSpeeds() {
    return new ChassisSpeeds(driveForwardVelocity(), driveLeftVelocity(), driveAngularVelocity());
  }
}
