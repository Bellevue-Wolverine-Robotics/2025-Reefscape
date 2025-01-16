package frc.robot.utils;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;

/** Sparkmax utility class */
public class SparkUtil {
  /** Stores whether any error has been detected by other utility methods */
  public static boolean sparkStickyFault = false;

  /**
   * Processes a value from a SparkMax only if the value is valid
   *
   * @param spark    Anything built on top of SparkBase, probably a SparkMax
   * @param supplier The supplier (method) that provides the value, as a double
   * @param consumer The consumer (function) that uses the value, as a double
   *
   */
  public static void ifOk(SparkBase spark, DoubleSupplier supplier, DoubleConsumer consumer) {
    double value = supplier.getAsDouble();
    if (spark.getLastError() == REVLibError.kOk) {
      consumer.accept(value);
    } else {
      sparkStickyFault = true;
    }
  }

  /**
   * Processes values from a SparkMax only if the values are valid
   *
   * @param spark     Anything built on top of SparkBase, probably a SparkMax
   * @param suppliers The suppliers (methods) that provides the values, as a
   *                  double
   * @param consumer  The consumer (function) that uses the values, as a double
   *
   */
  public static void ifOk(SparkBase spark, DoubleSupplier[] suppliers, Consumer<double[]> consumer) {
    double[] values = new double[suppliers.length];
    for (int i = 0; i < suppliers.length; i++) {
      values[i] = suppliers[i].getAsDouble();
      if (spark.getLastError() != REVLibError.kOk) {
        sparkStickyFault = true;
        return;
      }
    }
    consumer.accept(values);
  }

  /**
   * Attempts to run the command multiple times until no error is produced.
   *
   * @param spark       Anything built on top of SparkBase, typically a SparkMax
   * @param maxAttempts The maximum number of attempts the command will run
   * @param command     The command to try running
   */
  public static void tryUntilOk(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error == REVLibError.kOk) {
        break;
      } else {
        sparkStickyFault = true;
      }
    }
  }
}
