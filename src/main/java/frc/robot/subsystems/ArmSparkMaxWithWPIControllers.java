package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged(strategy = Strategy.OPT_IN)
public class ArmSparkMaxWithWPIControllers extends SubsystemBase {

  private final SparkMax m_leader;
  private final SparkMax m_follower;

  private final PIDController m_controller;
  private final ArmFeedforward m_feedforward;
  private final TrapezoidProfile m_profile;
  private final MutAngle m_targetAngle = Degrees.mutable(0);

  private final RelativeEncoder m_encoder;
  private final AbsoluteEncoder m_absoluteEncoder;

  private SparkMaxSim m_leaderSim;
  private SparkMaxSim m_followerSim;
  private SingleJointedArmSim m_sim;

  DoubleArrayPublisher m_targetStatePublisher = NetworkTableInstance.getDefault().getDoubleArrayTopic("Target State")
      .publish();
  DoubleArrayPublisher m_currentStatePublisher = NetworkTableInstance.getDefault().getDoubleArrayTopic("Current State")
      .publish();
  DoubleArrayPublisher m_intermediateStatePublisher = NetworkTableInstance.getDefault()
      .getDoubleArrayTopic("Intermediate State")
      .publish();
  DoublePublisher m_maxAnglePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Max Angle").publish();
  DoublePublisher m_ffVoltagePublisher = NetworkTableInstance.getDefault().getDoubleTopic("FF Voltage").publish();
  DoublePublisher m_pidVoltagePublisher = NetworkTableInstance.getDefault().getDoubleTopic("PID Voltage").publish();
  DoublePublisher m_totalVoltagePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Total Voltage").publish();

  public ArmSparkMaxWithWPIControllers(int leaderId, int followerId) {
    m_leader = new SparkMax(leaderId, MotorType.kBrushless);
    m_follower = new SparkMax(followerId, MotorType.kBrushless);

    m_encoder = m_leader.getEncoder();
    m_absoluteEncoder = m_leader.getAbsoluteEncoder();

    var constraints = new Constraints(DegreesPerSecond.of(100), DegreesPerSecondPerSecond.of(100));
    m_profile = new TrapezoidProfile(constraints);
    m_controller = new PIDController(5, 0.1, 0.1);
    m_feedforward = new ArmFeedforward(0.1, 0.1, 0.1);

    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();

    leaderConfig.idleMode(IdleMode.kBrake);
    followerConfig.idleMode(IdleMode.kBrake);

    leaderConfig.absoluteEncoder.positionConversionFactor(kGearing);
    followerConfig.follow(m_leader);

    m_leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    if (RobotBase.isSimulation()) {
      simulationInit();
    }
  }

  @Override
  public void periodic() {
    Angle currentPosition = getPosition();
    AngularVelocity currentVelocity = getVelocity();

    State currentState = new State(currentPosition, currentVelocity);
    State targetState = new State(getTargetPosition(), RotationsPerSecond.zero());

    State newState = m_profile.calculate(0.02, currentState, targetState);

    Angle newPosition = Radians.of(newState.position);
    // AngularVelocity newVelocity = RadiansPerSecond.of(newState.velocity);

    Voltage feedforwardVoltage = m_feedforward.calculate(currentPosition, currentVelocity);
    Voltage pidVoltage = Volts
        .of(m_controller.calculate(currentPosition.in(Radians), newPosition.in(Radians)));

    Voltage outputVoltage = feedforwardVoltage.plus(pidVoltage);

    m_ffVoltagePublisher.accept(feedforwardVoltage.in(Volts));
    m_pidVoltagePublisher.accept(pidVoltage.in(Volts));
    m_totalVoltagePublisher.accept(outputVoltage.in(Volts));
    m_currentStatePublisher.accept(new double[] { currentState.position, currentState.velocity });
    m_targetStatePublisher.accept(new double[] { targetState.position, targetState.velocity });
    m_intermediateStatePublisher.accept(new double[] { newState.position, newState.velocity });

    m_leader.setVoltage(outputVoltage);

    if (RobotBase.isSimulation()) {
      m_sim.setInputVoltage(getMotorVoltage().in(Volts));
    }
  }

  private double kGearing = 200.0;

  private void simulationInit() {
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

    m_leaderSim = new SparkMaxSim(m_leader, DCMotor.getNEO(1));
    m_followerSim = new SparkMaxSim(m_follower, DCMotor.getNEO(1));
  }

  @Override
  public void simulationPeriodic() {
    m_leaderSim.setBusVoltage(RobotController.getBatteryVoltage());
    m_followerSim.setBusVoltage(RobotController.getBatteryVoltage());

    m_sim.update(0.02);

    double velocity = RadiansPerSecond.of(m_sim.getVelocityRadPerSec()).in(RotationsPerSecond);

    m_leaderSim.iterate(velocity, RobotController.getBatteryVoltage(), 0.02);
    m_followerSim.iterate(velocity, RobotController.getBatteryVoltage(), 0.02);
  }

  @Logged
  public Voltage getSupplyVoltage() {
    return Volts.of(m_leader.getBusVoltage());
  }

  @Logged
  public double getAppliedOutput() {
    return m_leader.getAppliedOutput();
  }

  @Logged
  public Voltage getMotorVoltage() {
    return getSupplyVoltage().times(m_leader.getAppliedOutput());
  }

  @Logged
  public Temperature getMotorTemperature() {
    return Celsius.of(m_leader.getMotorTemperature());
  }

  @Logged
  public Angle getMotorPosition() {
    return Rotations.of(m_encoder.getPosition());
  }

  @Logged
  public AngularVelocity getMotorVelocity() {
    return RPM.of(m_encoder.getVelocity());
  }

  @Logged
  public Angle getPosition() {
    return Rotations.of(m_encoder.getPosition());
  }

  @Logged
  public AngularVelocity getVelocity() {
    return RPM.of(m_absoluteEncoder.getVelocity());
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
}
