package org.firstinspires.ftc.utilities;
import static java.lang.Math.floorMod;

public class PIDRingBuffer extends RingBuffer<PIDRingBuffer.DataPoint> {
    private double sum;
    private final int derivateBuffer;


    public PIDRingBuffer(int length, int derivateBuffer) {
        super(length, new DataPoint(0, System.currentTimeMillis()));
        if(length < 3 ){
            throw new UnsupportedOperationException("Integral Length must be more than two to support PIDRingBuffer");
        }
        this.derivateBuffer = derivateBuffer;

        sum = 0;
    }

    public PIDRingBuffer(int length) {
        this(length, 1);
    }

    /**
     * @return Returns DataPoint containing the integral (sum of all values in RingBuffer) and derivative (rate of change between last two values of RingBuffer where x is time and y is error)
     */
    public IntegralDerivativePair update(Double currentError, long currentTime) {
        System.out.println(list.size());
        int previousIndex = floorMod(index - derivateBuffer, list.size());
        System.out.println("Index" + index);
        DataPoint previousValue = list.get(previousIndex);
        double derivative = (currentError - previousValue.error) / (currentTime - previousValue.time);
        sum -= super.getValue(new DataPoint(currentError, currentTime)).error;
        System.out.println("sum" + sum);
        sum += currentError;
        System.out.println(sum);
        return new IntegralDerivativePair(sum, derivative);
    }

    @Override
public DataPoint getValue(DataPoint current) {
        return list.get(index);
    }

    protected static class DataPoint{
        private final double error;
        private final long time;

        public DataPoint(double error, long time) {
            this.error = error;
            this.time = time;
        }
    }

    public static class IntegralDerivativePair{
        public final double integral;
        public final double derivative;

        public IntegralDerivativePair(double integral, double derivative) {
            this.integral = integral;
            this.derivative = derivative;
        }
    }
}
