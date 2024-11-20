package frc.robot.commands.debug;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drive;

public class DebugCommands {
  public static Command brakeAndReset(Drive drive) {
    return Commands.sequence(
        // Brake Drivebase
        drive.stopCommand(),
        drive.setBrakeModeCommand(true),

        // Wait for robot to come to a stop
        Commands.waitSeconds(1.0),

        // Enable coast mode to easily move robot
        drive.setBrakeModeCommand(false)).ignoringDisable(true);
  }
}
