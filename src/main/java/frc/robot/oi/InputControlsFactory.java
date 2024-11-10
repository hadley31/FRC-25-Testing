package frc.robot.oi;

public class InputControlsFactory {
  private DriverControls m_driverControls;
  private OperatorControls m_operatorControls;

  private InputControlsFactory(DriverControls driverControls, OperatorControls operatorControls) {
    m_driverControls = driverControls;
    m_operatorControls = operatorControls;
  }

  public static InputControlsFactory determineInputs() {
    SingleUserXboxControls controls = new SingleUserXboxControls(0);
    return new InputControlsFactory(controls, controls);
  }

  public DriverControls getDriverControls() {
    return m_driverControls;
  }

  public OperatorControls getOperatorControls() {
    return m_operatorControls;
  }
}
