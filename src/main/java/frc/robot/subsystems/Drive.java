package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Drive extends SubsystemBase {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveDrivetrain m_swerve;
    private SwerveDriveState m_currentState = new SwerveDriveState();

    private final SwerveRequest.ApplyFieldSpeeds m_fieldCentricRequest = new SwerveRequest.ApplyFieldSpeeds();
    private final SwerveRequest.ApplyRobotSpeeds m_robotCentricRequest = new SwerveRequest.ApplyRobotSpeeds();

    public Drive(SwerveDrivetrain swerve) {
        m_swerve = swerve;
        m_swerve.registerTelemetry(state -> m_currentState = state.clone());

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public void driveFieldCentric(ChassisSpeeds speeds) {
        driveFieldCentric(speeds, false);
    }

    public void driveFieldCentric(ChassisSpeeds speeds, boolean openLoop) {
        var driveRequestType = openLoop
                ? SwerveModule.DriveRequestType.OpenLoopVoltage
                : SwerveModule.DriveRequestType.Velocity;
        m_swerve.setControl(m_fieldCentricRequest.withSpeeds(speeds).withDriveRequestType(driveRequestType));
    }

    public void driveRobotCentric(ChassisSpeeds speeds) {
        driveRobotCentric(speeds, false);
    }

    public void setControl(SwerveRequest request) {
        m_swerve.setControl(request);
    }

    public void driveRobotCentric(ChassisSpeeds speeds, boolean openLoop) {
        var driveRequestType = openLoop
                ? SwerveModule.DriveRequestType.OpenLoopVoltage
                : SwerveModule.DriveRequestType.Velocity;
        m_swerve.setControl(m_robotCentricRequest.withSpeeds(speeds).withDriveRequestType(driveRequestType));
    }

    public Pose2d getPose() {
        return m_currentState.Pose;
    }

    public void resetPose(Pose2d pose) {
        m_swerve.resetPose(pose);
    }

    public SwerveModuleState[] getModuleStates() {
        return m_currentState.ModuleStates;
    }

    public SwerveModulePosition[] getModulePositions() {
        return m_currentState.ModulePositions;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return m_currentState.Speeds;
    }

    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public Pose3d getPose3d() {
        return new Pose3d(getTranslation3d(), getRotation3d());
    }

    private Rotation3d getRotation3d() {
        return m_swerve.getRotation3d();
    }

    private Translation3d getTranslation3d() {
        var pose = getPose();
        return new Translation3d(pose.getX(), pose.getY(), 0);
    }

    public Command robotCentricDriveCommand(Supplier<ChassisSpeeds> robotCentricSpeeds) {
        return run(() -> driveRobotCentric(robotCentricSpeeds.get()));
    }

    public Command resetPoseCommand(Pose2d pose) {
        return runOnce(() -> resetPose(pose));
    }

    public Command stopCommand() {
        return runOnce(() -> driveRobotCentric(new ChassisSpeeds()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            m_swerve.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
