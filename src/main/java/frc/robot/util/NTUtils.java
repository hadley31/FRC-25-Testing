package frc.robot.util;

import java.util.EnumSet;
import java.util.function.Consumer;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.Subscriber;

public class NTUtils {
  public static DoubleSubscriber getDoubleSub(String name, double defaultValue, PubSubOption... options) {
    return NetworkTableInstance.getDefault().getDoubleTopic(name).subscribe(defaultValue, options);
  }

  public static DoubleEntry getDoubleEntry(String name, double defaultValue, PubSubOption... options) {
    var topic = NetworkTableInstance.getDefault().getDoubleTopic(name);
    return topic.getEntry(defaultValue, options);
  }

  public static void addSubListener(Subscriber sub, Consumer<Double> consumer) {
    NetworkTableInstance.getDefault().addListener(sub, EnumSet.of(Kind.kValueAll),
        (event) -> consumer.accept(event.valueData.value.getDouble()));
  }
}
