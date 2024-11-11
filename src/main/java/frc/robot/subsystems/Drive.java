package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;
import java.util.stream.Stream;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged(strategy = Strategy.OPT_IN)
public class Drive extends SubsystemBase {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  private final SwerveDrivetrain m_swerve;
  private SwerveDriveState m_currentState = new SwerveDriveState();

  private final SwerveRequest.ApplyFieldSpeeds m_fieldRelativeRequest = new SwerveRequest.ApplyFieldSpeeds();
  private final SwerveRequest.ApplyRobotSpeeds m_robotRelativeRequest = new SwerveRequest.ApplyRobotSpeeds();
  private final SwerveRequest.FieldCentricFacingAngle m_fieldRelativeFacingRequest = new FieldCentricFacingAngle();

  public Drive(SwerveDrivetrain swerve) {
    m_swerve = swerve;
    m_swerve.registerTelemetry(state -> m_currentState = state.clone());

    m_fieldRelativeFacingRequest.HeadingController.setPID(9, 0, 0);
    m_fieldRelativeFacingRequest.HeadingController.setTolerance(0.03);

    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  public void driveFieldRelative(ChassisSpeeds speeds) {
    driveFieldRelative(speeds, false);
  }

  public void driveFieldRelative(ChassisSpeeds speeds, boolean openLoop) {
    var driveRequestType = openLoop
        ? SwerveModule.DriveRequestType.OpenLoopVoltage
        : SwerveModule.DriveRequestType.Velocity;
    m_swerve.setControl(m_fieldRelativeRequest.withSpeeds(speeds).withDriveRequestType(driveRequestType));
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    driveRobotRelative(speeds, false);
  }

  public void setControl(SwerveRequest request) {
    m_swerve.setControl(request);
  }

  public void driveRobotRelative(ChassisSpeeds speeds, boolean openLoop) {
    var driveRequestType = openLoop
        ? SwerveModule.DriveRequestType.OpenLoopVoltage
        : SwerveModule.DriveRequestType.Velocity;
    m_swerve.setControl(m_robotRelativeRequest.withSpeeds(speeds).withDriveRequestType(driveRequestType));
  }

  public void driveRobotPointAtTarget(LinearVelocity xVelocity, LinearVelocity yVelocity, Translation2d target) {
    Pose2d expectedPose = getPose().exp(getChassisSpeeds().toTwist2d(0.02));
    Rotation2d targetRotation = target.minus(expectedPose.getTranslation()).getAngle();
    AngularVelocity feedForward = Radians.of(targetRotation.minus(getRotation()).getRadians()).divide(Seconds.of(0.3));

    m_swerve.setControl(m_fieldRelativeFacingRequest
        .withVelocityX(xVelocity)
        .withVelocityY(yVelocity)
        .withTargetDirection(targetRotation)
        .withTargetRateFeedforward(feedForward)
        .withRotationalDeadband(Units.DegreesPerSecond.of(1)));
  }

  @Logged
  public Pose2d getPose() {
    return m_currentState.Pose;
  }

  public void resetPose(Pose2d pose) {
    m_swerve.resetPose(pose);
  }

  @Logged
  public SwerveModuleState[] getModuleStates() {
    return m_currentState.ModuleStates;
  }

  @Logged
  public SwerveModuleState[] getDesiredModuleStates() {
    return Stream.of(m_swerve.getModules()).map(SwerveModule::getTargetState).toArray(SwerveModuleState[]::new);
  }

  @Logged
  public SwerveModulePosition[] getModulePositions() {
    return m_currentState.ModulePositions;
  }

  @Logged
  public ChassisSpeeds getChassisSpeeds() {
    return m_currentState.Speeds;
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  @Logged
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

  public Command robotRelativeDriveCommand(Supplier<ChassisSpeeds> robotRelativeSpeeds) {
    return run(() -> driveRobotRelative(robotRelativeSpeeds.get()));
  }

  public Command fieldRelativeDriveCommand(Supplier<ChassisSpeeds> robotRelativeSpeeds) {
    return run(() -> driveFieldRelative(robotRelativeSpeeds.get()));
  }

  public Command pointAtTargetDriveCommand(Supplier<LinearVelocity> xVelocity, Supplier<LinearVelocity> yVelocity,
      Translation2d target) {
    return run(() -> driveRobotPointAtTarget(xVelocity.get(), yVelocity.get(), target));
  }

  public Command resetPoseCommand(Pose2d pose) {
    return runOnce(() -> resetPose(pose));
  }

  public Command stopCommand() {
    return runOnce(() -> driveRobotRelative(new ChassisSpeeds()));
  }

  public Command seedFieldRelativeCommand() {
    return Commands.runOnce(m_swerve::seedFieldCentric);
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
