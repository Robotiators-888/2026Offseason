package frc.robot.utils;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.function.Function;
import org.opencv.core.Mat.Tuple2;

/**
 * Custom dynamic array implementation capable of returning trimmed raw arrays for SmartDashboard publishing.
 *
 * @param <T> Element type stored in the Vector.
 */
public class Vector<T> {
        // Stores the underlying array
        private T[] storage;
        // Stores the default value to instantiate new arrays
        private T defaultVal;
        // Stores the different sizes
        private int usedSize;
        private int size;
        // Defualt capacity of a vector
        private static final int DEFAULT_CAPACITY = 10;

        /**
         * Constructs a new Vector with the specified default element value.
         *
         * @param defaultValue Sample non-null default element value for array instantiation.
         */
        public Vector(T defaultValue) {
                defaultVal = defaultValue;
                size = DEFAULT_CAPACITY;
                usedSize = 0;
                storage = createArray(size, defaultVal);
        }

        /**
         * Helper method for instantiating a generic array of type T.
         *
         * @param size Array length.
         * @param defaultValue Default value to populate array entries.
         * @return Array of type T.
         */
        @SuppressWarnings("unchecked")
        private static <T> T[] createArray(int size, T defaultValue) {
                T[] array = (T[]) Array.newInstance(defaultValue.getClass(), size);
                Arrays.fill(array, defaultValue);
                return array;
        }

        /**
         * Appends an element to the back of the Vector, doubling capacity if needed.
         *
         * @param data Element to add.
         * @throws NullPointerException If passed data element is null.
         */
        public void add(T data) throws NullPointerException {
                if (data == null) {
                        throw new NullPointerException();
                } else {
                        if (usedSize + 1 < size) {
                                storage[usedSize] = data;
                                usedSize++;
                        } else {
                                size *= 2;
                                T[] tempStorage = storage;
                                storage = createArray(size, defaultVal);
                                int i = 0;
                                for (i = 0; i < tempStorage.length; i++) {
                                        storage[i] = tempStorage[i];
                                }
                                for (; i < storage.length; i++) {
                                        storage[i] = defaultVal;
                                }
                                storage[usedSize] = data;
                                usedSize++;
                        }
                }
        }

        /**
         * Retrieves the element at the specified index.
         *
         * @param index Array index.
         * @return Element at index.
         */
        public T get(int index) {
                return storage[index];
        }

        /**
         * Returns a trimmed array containing only populated elements.
         *
         * @return Array of type T[] with length equal to used element count.
         */
        public T[] toArray() {
                T[] result = createArray(usedSize, defaultVal);
                for (int i = 0; i < result.length; i++) {
                        result[i] = storage[i];
                }
                return result;
        }

        /**
         * Returns whether the Vector contains zero elements.
         *
         * @return True if empty, false otherwise.
         */
        public boolean isEmpty() {
                return usedSize == 0;
        }

        /**
         * Returns the number of elements currently stored in the Vector.
         *
         * @return Element count.
         */
        public int size() {
                return usedSize;
        }

        /**
         * Overwrites the element at the specified index.
         *
         * @param index Target index.
         * @param data New element value.
         */
        public void set(int index, T data) {
                storage[index] = data;
        }

        /**
         * Searches for the first element matching a custom predicate function.
         *
         * @param func Binary predicate function accepting a tuple of elements.
         * @param data Comparison target element.
         * @return Index of matching element, or -1 if not found.
         */
        public int findFirst(Function<Tuple2<T>, Boolean> func,
            T data) {
                for (int i = 0; i < usedSize; i++) {
                        if (func.apply(new Tuple2<T>(storage[i], data)))
                                return i;
                }
                return -1;
        }
}
