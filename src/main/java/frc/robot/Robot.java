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
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.AutoRoutines;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drive;
import frc.robot.util.ChoreoPathController;

@Logged
public class Robot extends TimedRobot {
  private Drive m_drive;
  private AutoChooser autoChooser;
  private Command m_autonomousCommand;

  public Robot() {
    DataLogManager.start();
    Epilogue.bind(this);

    m_drive = new Drive(TunerConstants.createDrivetrain());
    var pathController = new ChoreoPathController(m_drive::setControl);
    var factory = Choreo.createAutoFactory(m_drive, m_drive::getPose, pathController::followPath, this::isBlueAlliance,
        new AutoBindings());

    getCommandBindings().forEach(factory::bind);

    AutoRoutines routines = new AutoRoutines(m_drive);

    autoChooser = new AutoChooser(factory, "Choose Auto");

    autoChooser.addAutoRoutine("Test Auto", routines::testAuto);

    // m_drive.setDefaultCommand(m_drive.robotCentricDriveCommand(() -> new ChassisSpeeds(0.5, 0, 0)));
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

  private Map<String, Command> getCommandBindings() {
    return Map.of();
  }

  public boolean isBlueAlliance() {
    return DriverStation.getAlliance().orElse(null) == Alliance.Blue;
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
