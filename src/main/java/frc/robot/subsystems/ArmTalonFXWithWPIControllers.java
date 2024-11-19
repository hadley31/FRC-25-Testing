package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.NTUtils.getDoubleArrayPub;
import static frc.robot.util.NTUtils.getDoubleEntry;
import static frc.robot.util.NTUtils.getDoublePub;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.MutableArmFeedforward;
import frc.robot.util.NTUtils;

@Logged(strategy = Strategy.OPT_IN)
public class ArmTalonFXWithWPIControllers extends SubsystemBase {
  private final TalonFX m_leader;
  private final TalonFX m_follower;
  private final CANcoder m_remoteSensor;

  private final VoltageOut m_voltageControl = new VoltageOut(0).withEnableFOC(true);

  private final PIDController m_controller;
  private final MutableArmFeedforward m_feedforward;
  private final TrapezoidProfile m_profile;
  private final MutAngle m_targetAngle = Degrees.mutable(0);

  // Feedforward gain NT subscribers
  private final DoubleEntry m_ks = getDoubleEntry("Arm/FF/ks", 0);
  private final DoubleEntry m_kg = getDoubleEntry("Arm/FF/kg", 0);
  private final DoubleEntry m_kv = getDoubleEntry("Arm/FF/kv", 0);
  private final DoubleEntry m_ka = getDoubleEntry("Arm/FF/ka", 0);

  // PID gain NT subscribers
  private final DoubleEntry m_kp = getDoubleEntry("Arm/PID/kp", 0);
  private final DoubleEntry m_ki = getDoubleEntry("Arm/PID/ki", 0);
  private final DoubleEntry m_kd = getDoubleEntry("Arm/PID/kd", 0);

  private final DoubleArrayPublisher m_targetStatePublisher = getDoubleArrayPub("Arm/Target State");
  private final DoubleArrayPublisher m_currentStatePublisher = getDoubleArrayPub("Arm/Current State");
  private final DoubleArrayPublisher m_intermediateStatePublisher = getDoubleArrayPub("Intermediate State");
  private final DoublePublisher m_maxAnglePublisher = getDoublePub("Arm/Max Angle");
  private final DoublePublisher m_ffVoltagePublisher = getDoublePub("Arm/FF Voltage");
  private final DoublePublisher m_pidVoltagePublisher = getDoublePub("Arm/PID Voltage");
  private final DoublePublisher m_totalVoltagePublisher = getDoublePub("Arm/Total Voltage");

  private final DoubleEntry m_targetAngleEntry = getDoubleEntry("Arm/Target Angle", 0);

  private final Mechanism2d m_armMechanism2d = new Mechanism2d(50, 50);
  private final MechanismLigament2d m_armCurrentAngleLigament2d;
  private final MechanismLigament2d m_armTargetAngleLigament2d;

  // Sim Stuff
  private TalonFXSimState m_leaderSim;
  private TalonFXSimState m_followerSim;
  private CANcoderSimState m_remoteSensorSim;
  private SingleJointedArmSim m_sim;

  public ArmTalonFXWithWPIControllers(int leaderPort, int followerPort) {
    m_leader = new TalonFX(leaderPort, "rio");
    m_follower = new TalonFX(followerPort, "rio");
    m_remoteSensor = new CANcoder(60, "rio");

    var constraints = new Constraints(DegreesPerSecond.of(100), DegreesPerSecondPerSecond.of(100));
    m_profile = new TrapezoidProfile(constraints);
    m_controller = new PIDController(0, 0.0, 0.0);
    m_feedforward = new MutableArmFeedforward(0.0, 0, 0.0);

    m_armCurrentAngleLigament2d = new MechanismLigament2d("Arm", 20, m_targetAngle.in(Degrees), 3,
        new Color8Bit(Color.kGreen));
    m_armTargetAngleLigament2d = new MechanismLigament2d("Arm", 20, 0, 3, new Color8Bit(Color.kBlue));
    m_armMechanism2d.getRoot("Current Arm Root", 5, 10).append(m_armCurrentAngleLigament2d);
    m_armMechanism2d.getRoot("Target Arm Root", 5, 10).append(m_armTargetAngleLigament2d);
    SmartDashboard.putData("Arm Mechanism", m_armMechanism2d);

    FeedbackConfigs feedbackConfigs = new FeedbackConfigs();

    feedbackConfigs.RotorToSensorRatio = kGearing;
    feedbackConfigs.FeedbackRemoteSensorID = m_remoteSensor.getDeviceID();
    feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

    MotorOutputConfigs motorOutputConfig = new MotorOutputConfigs();

    motorOutputConfig.Inverted = InvertedValue.CounterClockwise_Positive;

    m_leader.getConfigurator().apply(motorOutputConfig, 0.05);
    m_follower.getConfigurator().apply(motorOutputConfig, 0.05);

    m_leader.getConfigurator().apply(feedbackConfigs, 0.05);
    m_follower.getConfigurator().apply(feedbackConfigs, 0.05);

    m_follower.setControl(new Follower(leaderPort, false));

    NTUtils.addSubListener(m_targetAngleEntry, degrees -> m_targetAngle.mut_replace(degrees, Degrees));

    m_targetAngleEntry.accept(20);

    if (Robot.isSimulation()) {
      simulationInit();
    }

    networkTablesInit();
  }

  @Override
  public void periodic() {
    Angle currentPosition = getPosition();
    AngularVelocity currentVelocity = getVelocity();

    Angle targetPosition = getTargetPosition();

    State currentState = new State(currentPosition, currentVelocity);
    State targetState = new State(targetPosition, RotationsPerSecond.zero());

    State newState = m_profile.calculate(0.02, currentState, targetState);

    Angle newPosition = Radians.of(newState.position);
    AngularVelocity newVelocity = RadiansPerSecond.of(newState.velocity);

    Voltage feedforwardVoltage = m_feedforward.calculate(currentPosition, currentVelocity);
    Voltage pidVoltage = Volts
        .of(m_controller.calculate(currentPosition.in(Radians), targetPosition.in(Radians)));

    Voltage outputVoltage = feedforwardVoltage.plus(pidVoltage);

    m_ffVoltagePublisher.accept(feedforwardVoltage.in(Volts));
    m_pidVoltagePublisher.accept(pidVoltage.in(Volts));
    m_totalVoltagePublisher.accept(outputVoltage.in(Volts));
    m_currentStatePublisher.accept(new double[] { currentState.position, currentState.velocity });
    m_targetStatePublisher.accept(new double[] { targetState.position, targetState.velocity });
    m_intermediateStatePublisher.accept(new double[] { newState.position, newState.velocity });

    m_leader.setControl(m_voltageControl.withOutput(outputVoltage));

    m_armCurrentAngleLigament2d.setAngle(getCurrentPositionRotation2d());
    m_armTargetAngleLigament2d.setAngle(m_targetAngle.in(Degrees));
  }

  @Logged
  public Voltage getSupplyVoltage() {
    return m_leader.getSupplyVoltage().getValue();
  }

  @Logged
  public Voltage getMotorVoltage() {
    return m_leader.getMotorVoltage().getValue();
  }

  @Logged
  public Temperature getMotorTemperature() {
    return m_leader.getDeviceTemp().getValue();
  }

  @Logged
  public Angle getMotorPosition() {
    return m_leader.getPosition().getValue();
  }

  @Logged
  public AngularVelocity getMotorVelocity() {
    return m_leader.getVelocity().getValue();
  }

  @Logged
  public Angle getPosition() {
    return m_remoteSensor.getAbsolutePosition().getValue();
  }

  @Logged
  public AngularVelocity getVelocity() {
    return m_remoteSensor.getVelocity().getValue();
  }

  @Logged
  public Angle getTargetPosition() {
    return m_targetAngle;
  }

  public Rotation2d getTargetPositionRotation2d() {
    return new Rotation2d(getTargetPosition());
  }

  public Rotation2d getCurrentPositionRotation2d() {
    return new Rotation2d(getPosition());
  }

  public void setTargetAngle(Angle angle) {
    m_targetAngle.mut_replace(angle);
  }

  public Command setTargetAngleCommand(Angle angle) {
    return runOnce(() -> setTargetAngle(angle));
  }

  public Command setTargetAngleCommand(Rotation2d angle) {
    return runOnce(() -> setTargetAngle(angle.getMeasure()));
  }

  public Command setTargetAngleCommand(Supplier<Angle> angleSupplier) {
    return run(() -> setTargetAngle(angleSupplier.get()));
  }

  private double kGearing = 200.0;

  public void simulationInit() {
    Distance armLength = Feet.of(2);
    Mass weight = Pounds.of(15);
    Angle minAngle = Degrees.of(0);
    Angle maxAngle = Degrees.of(80);
    m_maxAnglePublisher.accept(maxAngle.in(Radians));
    Angle startingAngle = Degrees.zero();
    boolean simulateGravity = true;
    MomentOfInertia moi = KilogramSquareMeters
        .of(SingleJointedArmSim.estimateMOI(armLength.in(Meters), weight.in(Kilograms)));

    m_sim = new SingleJointedArmSim(
        DCMotor.getNEO(2),
        kGearing,
        moi.in(KilogramSquareMeters),
        armLength.in(Meters),
        minAngle.in(Radians),
        maxAngle.in(Radians),
        simulateGravity,
        startingAngle.in(Radians),
        0.0001, 0.0001);

    m_leaderSim = m_leader.getSimState();
    m_followerSim = m_follower.getSimState();
    m_remoteSensorSim = m_remoteSensor.getSimState();

    m_leaderSim.Orientation = ChassisReference.CounterClockwise_Positive;
    m_followerSim.Orientation = ChassisReference.CounterClockwise_Positive;
  }

  private void networkTablesInit() {
    NTUtils.addSubListener(m_kp, m_controller::setP);
    NTUtils.addSubListener(m_ki, m_controller::setI);
    NTUtils.addSubListener(m_kd, m_controller::setD);

    NTUtils.addSubListener(m_ks, m_feedforward::setKs);
    NTUtils.addSubListener(m_kg, m_feedforward::setKg);
    NTUtils.addSubListener(m_kv, m_feedforward::setKv);
    NTUtils.addSubListener(m_ka, m_feedforward::setKa);

    m_kp.accept(m_kp.get());
    m_ki.accept(m_ki.get());
    m_kd.accept(m_kd.get());

    m_ks.accept(m_ks.get());
    m_kg.accept(m_kg.get());
    m_kv.accept(m_kv.get());
    m_ka.accept(m_ka.get());
  }

  @Override
  public void simulationPeriodic() {
    m_leaderSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_followerSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_remoteSensorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    var voltage = m_leaderSim.getMotorVoltage();

    m_sim.setInputVoltage(voltage);
    m_sim.update(0.02);

    // double batteryVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(m_sim.getCurrentDrawAmps());
    // RoboRioSim.setVInVoltage(batteryVoltage);

    m_remoteSensorSim.setRawPosition(Radians.of(m_sim.getAngleRads()));
    m_remoteSensorSim.setVelocity(RadiansPerSecond.of(m_sim.getVelocityRadPerSec()));

    m_leaderSim.setRawRotorPosition(targetAngleToMotor(m_sim.getAngleRads()));
    m_leaderSim.setRotorVelocity(targetAngleToMotor(m_sim.getVelocityRadPerSec()));

    m_followerSim.setRawRotorPosition(targetAngleToMotor(m_sim.getAngleRads()));
    m_followerSim.setRotorVelocity(targetAngleToMotor(m_sim.getVelocityRadPerSec()));
  }

  private double targetAngleToMotor(double radians) {
    return Units.radiansToRotations(radians) * kGearing;
  }
}
