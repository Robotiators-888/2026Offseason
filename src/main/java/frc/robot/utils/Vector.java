package frc.robot.utils;

import java.util.ArrayList;

/**
 * Clean wrapper for standard dynamic collections.
 */
@Deprecated
public class Vector<T> {
  private final ArrayList<T> list = new ArrayList<>();
  private final T defaultVal;

  public Vector(T defaultValue) {
    this.defaultVal = defaultValue;
  }

  public void add(T data) {
    list.add(data);
  }

  public T get(int index) {
    return list.get(index);
  }

  public boolean isEmpty() {
    return list.isEmpty();
  }

  public int size() {
    return list.size();
  }

  public void set(int index, T data) {
    list.set(index, data);
  }
}
