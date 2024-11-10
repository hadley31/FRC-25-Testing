package frc.robot.oi;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;

public class InputControlsFactory {
  private DriverControls m_driverControls;
  private OperatorControls m_operatorControls;

  private InputControlsFactory(DriverControls driverControls, OperatorControls operatorControls) {
    m_driverControls = driverControls;
    m_operatorControls = operatorControls;
  }

  /**
   * <b>NOTE:</b> This may not work unless it is called from the {@link Robot#driverStationConnected} method
   * @return
   */
  public static InputControlsFactory determineInputs() {
    int driverIndex = findFirstConnectedController(0);
    int operatorIndex = findFirstConnectedController(driverIndex + 1);

    boolean isDriverControllerConnected = driverIndex >= 0;
    boolean isOperatorControllerConnected = operatorIndex >= 0;

    boolean isDriverXboxControls = isDriverControllerConnected && DriverStation.getJoystickIsXbox(driverIndex);
    boolean isOperatorXboxControls = isOperatorControllerConnected && DriverStation.getJoystickIsXbox(operatorIndex);

    if (!isDriverControllerConnected && !isOperatorControllerConnected) {
      DriverStation.reportError("No controllers connected!!", false);
    }

    if (isDriverXboxControls && !isOperatorControllerConnected) {
      SingleUserXboxControls singleUserXboxControls = new SingleUserXboxControls(driverIndex);
      return new InputControlsFactory(singleUserXboxControls, singleUserXboxControls);
    }

    if (isDriverXboxControls && isOperatorXboxControls) {
      return new InputControlsFactory(new DriverXboxControls(driverIndex), new OperatorXboxControls(operatorIndex));
    }

    if (isDriverXboxControls && isOperatorControllerConnected && !isOperatorXboxControls) {
      return new InputControlsFactory(new DriverXboxControls(driverIndex), new OperatorPS5Controls(operatorIndex));
    }

    // Default to ports 0 and 1
    // return new InputControlsFactory(new DriverXboxControls(0), new OperatorXboxControls(1));
    SingleUserXboxControls singleUserXboxControls = new SingleUserXboxControls(0);
    return new InputControlsFactory(singleUserXboxControls, singleUserXboxControls);
  }

  public DriverControls getDriverControls() {
    return m_driverControls;
  }

  public OperatorControls getOperatorControls() {
    return m_operatorControls;
  }

  private static int findFirstConnectedController(int startIndex) {
    for (int i = startIndex; i < DriverStation.kJoystickPorts; i++) {
      if (DriverStation.isJoystickConnected(i)) {
        return i;
      }
    }

    return -1;
  }
}
