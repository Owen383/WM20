package org.firstinspires.ftc.utilities;

public class RingBufferOwen {

    private long[] array;
    private int i = 0;

    public RingBufferOwen(int length){
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