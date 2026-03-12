package frc.robot.utils;

import java.lang.reflect.Array;
import java.util.Arrays;

// This class was created to provide a dynamic array implementation that is able to return a regular array
// This is to ensure that we can easily pass arrays to SmartDashboard
public class Vector <T> {
  private T[] storage;
  private T defaultVal;
  private int usedSize;
  private int size;
  private static final int DEFAULT_CAPACITY = 10;

  public Vector (T defaultValue) {
    defaultVal = defaultValue;
    size = DEFAULT_CAPACITY;
    usedSize = 0;
    storage = createArray(size, defaultVal);
  }

  private static <T> T[] createArray(int size, T defaultValue) {
    // Terrible casting needed because of runtime objects and rules against generic arrays
    T[] array = (T[]) Array.newInstance(defaultValue.getClass(), size); // TODO: Is this okay?
    Arrays.fill(array, defaultValue);
    return array;
  }

  public void add (T data) throws NullPointerException {
    // Data shouldn't be null
    if (data == null) {
        throw new NullPointerException();
    }
    else {
      if (usedSize+1 < size) {
        // Inserts data
        storage[usedSize] = data;
        usedSize++;
      }
      else {
        size*=2;
        // Creates a reference to storage and keeps its data alive
        T[] tempStorage = storage;
        // Makes storage a new array
        storage = createArray(size, defaultVal);
        int i = 0;
        // "Copies" the elements (it really uses references but who cares) hopefully doesn't create memory leaks by keeping references to old allocations
        for (i = 0;i<tempStorage.length;i++) {
            storage[i] = tempStorage[i];
        }
        // Fills the rest of the array with the default value
        for (i=i;i<storage.length;i++) {
            storage[i] = defaultVal;
        }
        // Inserts data
        storage[usedSize] = data;
        usedSize++;
      }
    }
  }

  public T get (int index) {
    // I should make this throw something
    return storage[index];
  }

  public T[] toArray () {
    // Creates an array with minimal size and "copies" the data over once again it is really just references
    T[] result = createArray(usedSize, defaultVal);
    for (int i = 0;i<result.length;i++) {
        result[i] = storage[i];
    }
    return result;
  }

  public boolean isEmpty () {
    return usedSize == 0 ? true : false;
  }

  public int size () {
    return usedSize;
  }
}
