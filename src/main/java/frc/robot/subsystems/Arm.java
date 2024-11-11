package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Kilograms;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
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
import frc.robot.util.NTUtils;

@Logged(strategy = Strategy.OPT_IN)
public class Arm extends SubsystemBase {
  private final TalonFX m_leader;
  private final TalonFX m_follower;
  private final CANcoder m_remoteSensor;

  // Feedforward gain NT subscribers
  private final DoubleEntry m_ks = NTUtils.getDoubleEntry("Arm/FF/ks", 0);
  private final DoubleEntry m_kg = NTUtils.getDoubleEntry("Arm/FF/kg", 2.55);
  private final DoubleEntry m_kv = NTUtils.getDoubleEntry("Arm/FF/kv", 0);
  private final DoubleEntry m_ka = NTUtils.getDoubleEntry("Arm/FF/ka", 0);

  // PID gain NT subscribers
  private final DoubleEntry m_kp = NTUtils.getDoubleEntry("Arm/PID/kp", 900);
  private final DoubleEntry m_ki = NTUtils.getDoubleEntry("Arm/PID/ki", 0);
  private final DoubleEntry m_kd = NTUtils.getDoubleEntry("Arm/PID/kd", 0);

  private final DoubleEntry m_targetAngleEntry = NTUtils.getDoubleEntry("Arm/Target Angle", 0);

  private final Mechanism2d m_armMechanism2d = new Mechanism2d(50, 50);
  private final MechanismLigament2d m_armCurrentAngleLigament2d;
  private final MechanismLigament2d m_armTargetAngleLigament2d;

  private Rotation2d m_targetAngle = Rotation2d.fromDegrees(0);
  private final PositionVoltage positionControl = new PositionVoltage(0)
      .withEnableFOC(true)
      .withSlot(0)
      .withOverrideBrakeDurNeutral(false)
      .withUpdateFreqHz(100);

  private final Slot0Configs m_gains = new Slot0Configs();

  // Sim Stuff
  private TalonFXSimState m_leaderSim;
  private TalonFXSimState m_followerSim;
  private CANcoderSimState m_remoteSensorSim;
  private SingleJointedArmSim m_sim;

  public Arm(int leaderPort, int followerPort) {
    m_leader = new TalonFX(leaderPort, "rio");
    m_follower = new TalonFX(followerPort, "rio");
    m_remoteSensor = new CANcoder(60, "rio");

    m_armCurrentAngleLigament2d = new MechanismLigament2d("Arm", 20, m_targetAngle.getDegrees(), 3,
        new Color8Bit(Color.kGreen));
    m_armTargetAngleLigament2d = new MechanismLigament2d("Arm", 20, 0, 3, new Color8Bit(Color.kBlue));
    m_armMechanism2d.getRoot("Current Arm Root", 5, 10).append(m_armCurrentAngleLigament2d);
    m_armMechanism2d.getRoot("Target Arm Root", 5, 10).append(m_armTargetAngleLigament2d);
    SmartDashboard.putData("Arm Mechanism", m_armMechanism2d);

    m_gains.GravityType = GravityTypeValue.Arm_Cosine;

    m_gains.kS = m_ks.get();
    m_gains.kG = m_kg.get();
    m_gains.kV = m_kv.get();
    m_gains.kA = m_ka.get();

    m_gains.kP = m_kp.get();
    m_gains.kI = m_ki.get();
    m_gains.kD = m_kd.get();

    FeedbackConfigs feedbackConfigs = new FeedbackConfigs();

    feedbackConfigs.RotorToSensorRatio = kGearing;
    feedbackConfigs.FeedbackRemoteSensorID = m_remoteSensor.getDeviceID();
    feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

    MotorOutputConfigs motorOutputConfig = new MotorOutputConfigs();

    motorOutputConfig.Inverted = InvertedValue.CounterClockwise_Positive;

    m_leader.getConfigurator().apply(m_gains, 0.05);
    m_follower.getConfigurator().apply(m_gains, 0.05);

    m_leader.getConfigurator().apply(motorOutputConfig, 0.05);
    m_follower.getConfigurator().apply(motorOutputConfig, 0.05);

    m_leader.getConfigurator().apply(feedbackConfigs, 0.05);
    m_follower.getConfigurator().apply(feedbackConfigs, 0.05);

    m_follower.setControl(new Follower(leaderPort, false));

    NTUtils.addSubListener(m_ks, updateGains(ks -> m_gains.kS = ks));
    NTUtils.addSubListener(m_kg, updateGains(kg -> m_gains.kG = kg));
    NTUtils.addSubListener(m_kv, updateGains(kv -> m_gains.kV = kv));
    NTUtils.addSubListener(m_ka, updateGains(ka -> m_gains.kA = ka));

    NTUtils.addSubListener(m_kp, updateGains(kp -> m_gains.kP = kp));
    NTUtils.addSubListener(m_ki, updateGains(ki -> m_gains.kI = ki));
    NTUtils.addSubListener(m_kd, updateGains(kd -> m_gains.kD = kd));

    NTUtils.addSubListener(m_targetAngleEntry, degrees -> m_targetAngle = Rotation2d.fromDegrees(degrees));

    m_targetAngleEntry.accept(20);

    if (Robot.isSimulation()) {
      simulationInit();
    }
  }

  @Override
  public void periodic() {
    m_leader.setControl(positionControl.withPosition(m_targetAngle.getRotations()));

    m_armCurrentAngleLigament2d.setAngle(getAngle());
    m_armTargetAngleLigament2d.setAngle(m_targetAngle);
  }

  public void setTargetAngle(Rotation2d target) {
    m_targetAngle = target;
  }

  @Logged
  public Rotation2d getTargetAngle() {
    return m_targetAngle;
  }

  @Logged
  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(m_leader.getPosition().getValueAsDouble());
  }

  public Command setTargetAngleCommand(Rotation2d targetAngle) {
    return runOnce(() -> setTargetAngle(targetAngle));
  }

  public Command setTargetAngleCommand(Supplier<Rotation2d> supplier) {
    return run(() -> setTargetAngle(supplier.get()));
  }

  private double kGearing = 200.0;

  public void simulationInit() {
    double armLength = Units.feetToMeters(2);
    double minAngleRad = Units.degreesToRadians(0);
    double maxAngleRad = Units.degreesToRadians(80);
    boolean simulateGravity = true;
    double moi = SingleJointedArmSim.estimateMOI(armLength, Kilograms.of(8).magnitude());
    m_sim = new SingleJointedArmSim(
        DCMotor.getFalcon500Foc(2),
        kGearing,
        moi,
        armLength,
        minAngleRad,
        maxAngleRad,
        simulateGravity,
        Units.degreesToRadians(0),
        0.0001, 0.0001);
    m_leaderSim = m_leader.getSimState();
    m_followerSim = m_follower.getSimState();
    m_remoteSensorSim = m_remoteSensor.getSimState();

    m_leaderSim.Orientation = ChassisReference.CounterClockwise_Positive;
    m_followerSim.Orientation = ChassisReference.CounterClockwise_Positive;
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

    m_remoteSensorSim.setRawPosition(Units.radiansToRotations(m_sim.getAngleRads()));
    m_remoteSensorSim.setVelocity(Units.radiansToRotations(m_sim.getVelocityRadPerSec()));

    m_leaderSim.setRawRotorPosition(targetAngleToMotor(m_sim.getAngleRads()));
    m_leaderSim.setRotorVelocity(targetAngleToMotor(m_sim.getVelocityRadPerSec()));

    m_followerSim.setRawRotorPosition(targetAngleToMotor(m_sim.getAngleRads()));
    m_followerSim.setRotorVelocity(targetAngleToMotor(m_sim.getVelocityRadPerSec()));
  }

  private Consumer<Double> updateGains(Consumer<Double> update) {
    return (value) -> {
      update.accept(value);
      m_leader.getConfigurator().apply(m_gains, 0.05);
      m_follower.getConfigurator().apply(m_gains, 0.05);
      System.out.println("Updated arm gains:\n" + m_gains);
    };
  }

  private double targetAngleToMotor(double radians) {
    return Units.radiansToRotations(radians) * kGearing;
  }
}
