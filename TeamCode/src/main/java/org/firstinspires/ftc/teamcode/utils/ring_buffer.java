package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.data.dataStorage;

import java.lang.reflect.Array;

public class ring_buffer<T> {
    final int DEFAULT_CAPACITY = 8;
    private final int capacity;
    T[] buffer;
    int readIndex = 0;
    int writeIndex = -1;

    public ring_buffer(int capacity)
    {
        this.capacity = capacity;
        this.buffer = (T[]) new Object[capacity];
    }
    public ring_buffer(){
        this.capacity = DEFAULT_CAPACITY;
        this.buffer = (T[]) new Object[DEFAULT_CAPACITY];
    }

    public void put(T element){
        buffer[++writeIndex % capacity] = element;
        readIndex = writeIndex; /* to read the oldest value first */
    }

    public boolean offer(T element){
        boolean isFull = writeIndex > capacity;

        if (isFull)
            return false;
        put(element);
        return true;
    }

    public T get(){
        return buffer[readIndex++ % capacity];
    }

    public int getCapacity() {
        return capacity;
    }

    public void output() {
        int index = 0;
        for (T element : this.buffer) {
            if (element != null) {
                dataStorage.DSTelemetry.addData(index + "", element.toString());
                index++;
            }
        }
        dataStorage.DSTelemetry.update();
    }

    public void dump() {
        int index = 0;
        for (@NonNull T element : this.buffer) {
            dataStorage.DSTelemetry.addData("dumped " + index + "", element.toString());
            index++;
        }
        dataStorage.DSTelemetry.update();
    }
}
