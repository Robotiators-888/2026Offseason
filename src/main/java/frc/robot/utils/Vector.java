package frc.robot.utils;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.function.Function;

import org.opencv.core.Mat.Tuple2;

// This class was created to provide a dynamic array implementation that is able to return a regular array
// This is to ensure that we can easily pass arrays to SmartDashboard
public class Vector <T> {
  // Stores the underlying array
  private T[] storage;
  // Stores the default value to instantiate new arrays
  private T defaultVal;
  // Stores the different sizes
  private int usedSize;
  private int size;
  // Defualt capacity of a vector
  private static final int DEFAULT_CAPACITY = 10;

  // Constructor takes a default value because of java's weird generics
  public Vector (T defaultValue) {
    defaultVal = defaultValue;
    size = DEFAULT_CAPACITY;
    usedSize = 0;
    storage = createArray(size, defaultVal);
  }

  // Creates an array with a size and a default value with a type of T
  @SuppressWarnings("unchecked")
  private static <T> T[] createArray(int size, T defaultValue) {
    // Terrible casting needed because of runtime objects and rules against generic arrays
    T[] array = (T[]) Array.newInstance(defaultValue.getClass(), size);
    Arrays.fill(array, defaultValue);
    return array;
  }

  // Adds more data to the back of the Vector
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
        for (;i<storage.length;i++) {
            storage[i] = defaultVal;
        }
        // Inserts data
        storage[usedSize] = data;
        usedSize++;
      }
    }
  }

  // Gets a value from the Vector
  public T get (int index) {
    // I should make this throw something
    return storage[index];
  }

  // Returns an array of type T[] from the contents of the Vector without the unused space
  public T[] toArray () {
    // Creates an array with minimal size and "copies" the data over once again it is really just references
    T[] result = createArray(usedSize, defaultVal);
    for (int i = 0;i<result.length;i++) {
        result[i] = storage[i];
    }
    return result;
  }

  // Returns if the Vector is empty
  public boolean isEmpty () {
    return usedSize == 0;
  }

  // Returns the actual used size of the Vector
  public int size () {
    return usedSize;
  }

  // Sets and index to a value
  public void set (int index, T data) {
    storage[index] = data;
  }

  // Finds the first instance of something in the Vector
  // Takes a function that is meant to compare to values of type T and return if they are equal or not
  // Also takes T data which is the value to find
  // Returns the index or -1 if not found, it should return an optional but doing it c style kind of makes sense for the use case
  public int findFirst (Function<Tuple2<T>, Boolean> func, T data) { // Maybe I could just use .equals but this is way cooler and may not always be overrided
    for (int i = 0; i < usedSize; i++) {
      if (func.apply(new Tuple2<T>(storage[i], data)))
        return i;
    }
    return -1;
  }
}
