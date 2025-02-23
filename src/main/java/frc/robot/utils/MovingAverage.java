package frc.robot.utils;

import java.util.LinkedList;
import java.util.Queue;

public class MovingAverage {

  private final int size;
  private final Queue<Double> window;
  private double sum;

  public MovingAverage(int size) {
    this.size = size;
    this.window = new LinkedList<>();
    this.sum = 0.0;
  }

  public double next(double value) {
    if (window.size() == size) {
      sum -= window.poll();
    }
    window.offer(value);
    sum += value;
    return sum / window.size();
  }
}
