package frc.robot.commands.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoLoop;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drive;

public class AutoRoutines {
    private final Drive m_drive;

    public AutoRoutines(Drive drive) {
        m_drive = drive;
    }

    public Command testAuto(AutoFactory factory) {
        final AutoLoop routine = factory.newLoop("Test Auto");
        final AutoTrajectory testPath = factory.trajectory("TestPath", routine);

        routine.enabled().onTrue(
            Commands.print("STARTING")
            .andThen(resetPoseCommand(routine, testPath))
            .andThen(Commands.print("AHHHHHHH"))
            .andThen(testPath.cmd())
            .andThen(Commands.print("DONE")));

        return routine.cmd();
    }

    private Command resetPoseCommand(AutoLoop loop, AutoTrajectory path) {
        return m_drive.runOnce(() -> path.getInitialPose().ifPresentOrElse(
                pose -> m_drive.resetPose(pose),
                loop::kill));
    }
}
