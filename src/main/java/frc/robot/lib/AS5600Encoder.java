package frc.robot.lib;

import edu.wpi.first.wpilibj.MedianFilter;
import com.ctre.phoenix.motorcontrol.SensorCollection;

/** * Reads PWM values from the AS5600. */
public class AS5600Encoder {
    private final SensorCollection sensors;
    private volatile int lastValue = Integer.MIN_VALUE;

    private MedianFilter filter = new MedianFilter(50);

    public AS5600Encoder(SensorCollection sensors) {
        this.sensors = sensors;
    }

    public int getPwmPosition() {
        int raw = sensors.getPulseWidthRiseToFallUs();
        if (raw == 0) {
            int lastValue = (int)filter.calculate(this.lastValue);
            if (lastValue == Integer.MIN_VALUE) {
                return 0;
            }
            return lastValue;
        }
        int actualValue = (int)filter.calculate(Math.min(4096, raw - 128));
        lastValue = actualValue;
        return actualValue;
    }
}