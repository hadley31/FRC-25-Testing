package frc.robot.util;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Drive;

public class DriveSysIdRoutineFactory {
  public enum DriveSysIdRoutineType {
    kTranslation,
    kSteer,
    kRotation
  }

  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

  private final SysIdRoutine m_routine;

  private final Drive m_drive;
  private final DriveSysIdRoutineType m_routineType;

  public DriveSysIdRoutineFactory(Drive drive, DriveSysIdRoutineType type) {
    m_drive = drive;
    m_routineType = type;

    m_routine = createRoutine();
  }

  private SysIdRoutine createRoutine() {
    switch (m_routineType) {
      case kTranslation:
        return createSysIdRoutineTranslation();
      case kSteer:
        return createSysIdRoutineSteer();
      case kRotation:
        return createSysIdRoutineRotation();
      default:
        DriverStation.reportError("Invalid DriveSysIdRoutineType selected. Defaulting to Translation.", true);
        return createSysIdRoutineTranslation();
    }
  }

  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  private SysIdRoutine createSysIdRoutineTranslation() {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            null, // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
        new SysIdRoutine.Mechanism(
            output -> m_drive.setControl(m_translationCharacterization.withVolts(output)),
            null,
            m_drive));
  }

  /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
  private SysIdRoutine createSysIdRoutineSteer() {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            null, // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
        new SysIdRoutine.Mechanism(
            volts -> m_drive.setControl(m_steerCharacterization.withVolts(volts)),
            null,
            m_drive));
  }

  private SysIdRoutine createSysIdRoutineRotation() {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
        new SysIdRoutine.Mechanism(
            output -> {
              /* output is actually radians per second, but SysId only supports "volts" */
              m_drive.setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
              /* also log the requested output for SysId */
              SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            m_drive));
  }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine
   * specified by {@link #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_routine.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine
   * specified by {@link #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_routine.dynamic(direction);
  }
}
