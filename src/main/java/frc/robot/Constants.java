package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {
  public static class DriveConstants {
    public static final LinearVelocity kMaxLinearSpeed = Units.MetersPerSecond.of(5);
    public static final AngularVelocity kMaxAngularSpeed = Units.RotationsPerSecond.of(2);
  }
}
