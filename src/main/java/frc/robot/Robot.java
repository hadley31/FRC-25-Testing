// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import choreo.Choreo;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory.AutoBindings;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.auto.AutoRoutines;
import frc.robot.generated.TunerConstants;
import frc.robot.oi.DriverControls;
import frc.robot.oi.InputControlsFactory;
import frc.robot.oi.OperatorControls;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.util.ChoreoPathController;
import frc.robot.util.DriveSysIdRoutineFactory;
import frc.robot.util.DriveSysIdRoutineFactory.DriveSysIdRoutineType;

@Logged(strategy = Strategy.OPT_IN)
public class Robot extends TimedRobot {
  @Logged
  private Drive m_drive;
  @Logged
  private Arm m_arm;
  private AutoChooser autoChooser;
  private Command m_autonomousCommand;

  private DriverControls m_driverControls;
  private OperatorControls m_operatorControls;

  public Robot() {
    configureLogging();
    configureSubsystems();
    configureBindings();
    configureAutos();
  }

  private void configureLogging() {
    DataLogManager.start();
    Epilogue.bind(this);
  }

  private void configureSubsystems() {
    m_drive = new Drive(TunerConstants.DriveTrain);
    // m_arm = new Arm(17, 18);
  }

  private void configureBindings() {
    System.out.println("Configuring button bindings.");
    CommandScheduler.getInstance().getDefaultButtonLoop().clear();

    InputControlsFactory controlsFactory = InputControlsFactory.determineInputs();
    m_driverControls = controlsFactory.getDriverControls();
    m_operatorControls = controlsFactory.getOperatorControls();

    Command fieldRelativeDriveCommand = m_drive.fieldRelativeDriveCommand(m_driverControls::getTargetChassisSpeeds);
    Command robotRelativeDriveCommand = m_drive.robotRelativeDriveCommand(m_driverControls::getTargetChassisSpeeds);
    Command pointAtSpeakerDriveCommand = m_drive.pointAtTargetDriveCommand(m_driverControls::driveForwardVelocity,
        m_driverControls::driveLeftVelocity, FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d());
    Command seedFieldRelativeCommand = m_drive.seedFieldRelativeCommand();

    m_drive.setDefaultCommand(fieldRelativeDriveCommand);

    m_driverControls.robotRelativeDrive().whileTrue(robotRelativeDriveCommand);
    m_driverControls.faceSpeakerDrive().whileTrue(pointAtSpeakerDriveCommand);
    m_driverControls.seedFieldRelative().onTrue(seedFieldRelativeCommand);

    // m_operatorControls.stow().onTrue(m_arm.setTargetAngleCommand(Rotation2d.kZero));
    // m_operatorControls.scoreArm45Deg().onTrue(m_arm.setTargetAngleCommand(Rotation2d.fromDegrees(45)));
    // m_operatorControls.scoreArm75Deg().onTrue(m_arm.setTargetAngleCommand(Rotation2d.fromDegrees(75)));

    DriveSysIdRoutineFactory sysIdRoutineFactory = new DriveSysIdRoutineFactory(m_drive,
        DriveSysIdRoutineType.kTranslation);
    m_operatorControls.quasistaticForward().whileTrue(sysIdRoutineFactory.sysIdQuasistatic(Direction.kForward));
    m_operatorControls.quasistaticReverse().whileTrue(sysIdRoutineFactory.sysIdQuasistatic(Direction.kReverse));
    m_operatorControls.dynamicForward().whileTrue(sysIdRoutineFactory.sysIdDynamic(Direction.kForward));
    m_operatorControls.dynamicReverse().whileTrue(sysIdRoutineFactory.sysIdDynamic(Direction.kReverse));

    m_operatorControls.ampScoreRev().whileTrue(Commands.print("REVING AT AMP SPEED"));
    m_operatorControls.speakerRev().whileTrue(Commands.print("REVING AT SPEAKER SPEED"));

    m_operatorControls.ampScoreRev().and(m_operatorControls.shoot()).whileTrue(Commands.print("SHOOT AMP"));
    m_operatorControls.speakerRev().and(m_operatorControls.shoot()).whileTrue(Commands.print("SHOOT SPEAKER"));
  }

  private void configureAutos() {
    var pathController = new ChoreoPathController(m_drive::setControl);
    var factory = Choreo.createAutoFactory(m_drive, m_drive::getPose, pathController::followPath, this::isBlueAlliance,
        new AutoBindings());

    getCommandBindings().forEach(factory::bind);

    AutoRoutines routines = new AutoRoutines(m_drive);

    autoChooser = new AutoChooser(factory, "Choose Auto");

    autoChooser.addAutoRoutine("Test Auto", routines::testAuto);
  }

  @Override
  public void robotPeriodic() {
    autoChooser.update();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = autoChooser.getSelectedAutoRoutine();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void driverStationConnected() {
    configureBindings();
  }

  private Map<String, Command> getCommandBindings() {
    return Map.of(
        "PrintHi", Commands.print("Hi!"),
        "PrintHello", Commands.print("Hello!"),
        "PrintEnd", Commands.print("End!"));
    // "Arm0", m_arm.setTargetAngleCommand(Rotation2d.fromDegrees(0)),
    // "Arm45", m_arm.setTargetAngleCommand(Rotation2d.fromDegrees(45)),
    // "Arm75", m_arm.setTargetAngleCommand(Rotation2d.fromDegrees(75)));
  }

  public boolean isBlueAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
