package org.firstinspires.ftc.teamcode.owen;

public class RingBuffer {

    private long[] array;
    private int i = 0;

    public RingBuffer(int length){
        array = new long [length];
    }

    public long getValue(long current){
        long retVal = array[i];

        array[i] = current;

        i++;
        i = i % array.length;

        return retVal;

    }

}