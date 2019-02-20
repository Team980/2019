package com.team980.robot2019.sensors;

import edu.wpi.first.wpilibj.I2C;

import java.nio.ByteBuffer;

/**
 * Communicates with a Rioduino connected to the roboRIO via MXP.
 * Shares the received data with very easy to use getter methods,
 * and provides a id-based command interface to set its properties.
 */
@Deprecated //Not the final schema!
public final class Rioduino {

    private static final int DEVICE_ADDRESS = 10;
    private static final int BUFFER_SIZE = 4;

    private I2C i2C;

    private short targetCenterCoord;
    private short targetWidth;

    public Rioduino() {
        i2C = new I2C(I2C.Port.kMXP, DEVICE_ADDRESS);
    }

    public void updateData() {
        ByteBuffer buffer = ByteBuffer.allocate(BUFFER_SIZE);

        i2C.readOnly(buffer, BUFFER_SIZE);

        buffer.position(0);

        targetCenterCoord = buffer.getShort();
        targetWidth = buffer.getShort();
    }

    /**
     * <p>The center x-coordinate of the detected vision targets</p>
     * <p>Ranges from zero to 319</p>
     */
    public double getTargetCenterCoord() {
        return targetCenterCoord;
    }

    /**
     * <p>The combined width of the vision targets, in pixels</p>
     */
    public double getTargetWidth() {
        return targetWidth;
    }
}
