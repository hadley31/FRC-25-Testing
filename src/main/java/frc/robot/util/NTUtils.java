package frc.robot.util;

import java.util.EnumSet;
import java.util.function.Consumer;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.Subscriber;

public class NTUtils {
  public static DoubleTopic getDoubleTopic(String name) {
    return NetworkTableInstance.getDefault().getDoubleTopic(name);
  }

  public static DoubleArrayTopic getDoubleArrayTopic(String name) {
    return NetworkTableInstance.getDefault().getDoubleArrayTopic(name);
  }

  public static DoubleSubscriber getDoubleSub(String name, double defaultValue, PubSubOption... options) {
    return getDoubleTopic(name).subscribe(defaultValue, options);
  }

  public static DoubleArraySubscriber getDoubleArraySub(String name, double[] defaultValue, PubSubOption... options) {
    return getDoubleArrayTopic(name).subscribe(defaultValue, options);
  }

  public static DoubleEntry getDoubleEntry(String name, double defaultValue, PubSubOption... options) {
    return getDoubleTopic(name).getEntry(defaultValue, options);
  }

  public static DoublePublisher getDoublePub(String name, PubSubOption... options) {
    return getDoubleTopic(name).publish(options);
  }

  public static DoubleArrayPublisher getDoubleArrayPub(String name, PubSubOption... options) {
    return getDoubleArrayTopic(name).publish(options);
  }

  public static void addSubListener(Subscriber sub, Consumer<Double> consumer) {
    NetworkTableInstance.getDefault().addListener(sub, EnumSet.of(Kind.kValueAll),
        (event) -> consumer.accept(event.valueData.value.getDouble()));
  }
}
