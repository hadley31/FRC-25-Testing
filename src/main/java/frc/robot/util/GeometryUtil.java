package frc.robot.util;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.numbers.N3;

public class GeometryUtil {
  public static final Vector<N3> kUp = VecBuilder.fill(0, 0, 1);
  public static final Vector<N3> kLeft = VecBuilder.fill(0, 1, 0);
  public static final Vector<N3> kForward = VecBuilder.fill(1, 0, 0);

  public static Vector<N3> vector(Quaternion q) {
    return VecBuilder.fill(q.getX(), q.getY(), q.getZ());
  }

  public static Vector<N3> rotateVector(Vector<N3> v, Quaternion q) {
    // https://stackoverflow.com/a/58077034
    // v' = v + 2 * r x (s * v + r x v) / m
    double s = q.getW();
    double x = q.getX();
    double y = q.getY();
    double z = q.getZ();
    Vector<N3> r = vector(q);
    var sv = v.times(s);
    Vector<N3> rxv = Vector.cross(r, v);
    Vector<N3> rxsvrxv = Vector.cross(r, sv.plus(rxv));
    double m = s * s + x * x + y * y + z * z;
    return v.plus(rxsvrxv.times(2 / m));
  }
}
