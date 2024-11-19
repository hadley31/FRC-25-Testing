package frc.robot.util;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class MutableArmFeedforward {
  private ArmFeedforward m_feedforward;

  public MutableArmFeedforward(double ks, double kg, double kv) {
    m_feedforward = new ArmFeedforward(ks, kg, kv);
  }

  public MutableArmFeedforward(double ks, double kg, double kv, double ka) {
    m_feedforward = new ArmFeedforward(ks, kg, kv, ka);
  }

  public void setKs(double ks) {
    m_feedforward = new ArmFeedforward(ks, getKg(), getKv(), getKa());
  }

  public void setKg(double kg) {
    m_feedforward = new ArmFeedforward(getKs(), kg, getKv(), getKa());
  }

  public void setKv(double kv) {
    m_feedforward = new ArmFeedforward(getKs(), getKg(), kv, getKa());
  }

  public void setKa(double ka) {
    m_feedforward = new ArmFeedforward(getKs(), getKg(), getKv(), ka);
  }

  public double getKs() {
    return m_feedforward.getKs();
  }

  public double getKg() {
    return m_feedforward.getKg();
  }

  public double getKv() {
    return m_feedforward.getKv();
  }

  public double getKa() {
    return m_feedforward.getKa();
  }

  public Voltage calculate(Angle currentAngle, AngularVelocity currentVelocity) {
    return m_feedforward.calculate(currentAngle, currentVelocity);
  }

  public Voltage calculate(Angle currentAngle, AngularVelocity currentVelocity, AngularVelocity nextVelocity) {
    return m_feedforward.calculate(currentAngle, currentVelocity, nextVelocity);
  }
}
