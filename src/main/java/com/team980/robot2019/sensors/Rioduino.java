package com.team980.robot2019.sensors;

import com.team980.robot2019.vision.VisionDataProvider;
import edu.wpi.first.wpilibj.I2C;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * Communicates with a Rioduino connected to the roboRIO via MXP.
 * Shares the received data with very easy to use getter methods,
 * and provides a id-based command interface to set its properties.
 * <p>
 * Implements VisionDataProvider to provide data from the front Pixy, which was already calculated offboard.
 */
public final class Rioduino implements VisionDataProvider {

    private static final int DEVICE_ADDRESS = 10;
    private static final int BUFFER_SIZE = 28;

    private I2C i2C;

    // Absolute Encoders
    private int shoulderValue;
    private int elbowValue;
    private int wristValue;

    private float shoulderVelocity;
    private float elbowVelocity;
    private float wristVelocity;

    // Front Pixy
    private short targetCenterCoord;
    private short targetWidth;

    public Rioduino() {
        i2C = new I2C(I2C.Port.kMXP, DEVICE_ADDRESS);
    }

    public void updateData() {
        ByteBuffer buffer = ByteBuffer.allocate(BUFFER_SIZE);

        i2C.readOnly(buffer, BUFFER_SIZE);

        buffer.position(0);

        shoulderValue = buffer.getInt();
        elbowValue = buffer.getInt();
        wristValue = buffer.getInt();

        targetCenterCoord = buffer.getShort();
        targetWidth = buffer.getShort();

        buffer.order(ByteOrder.LITTLE_ENDIAN);

        shoulderVelocity = buffer.getFloat();
        elbowVelocity = buffer.getFloat();
        wristVelocity = buffer.getFloat();
    }

    /*public void sendCommand(Command command) {
        ByteBuffer buffer = ByteBuffer.allocate(1);
        buffer.put(command.id);
        i2C.writeBulk(buffer, 1);
    }*/

    @Override
    public String getSource() {
        return "Rioduino";
    }

    /**
     * <p>The angle of the arm's shoulder joint</p>
     * <p>Ranges from zero to 360</p>
     */
    public float getShoulderAngle() {
        var angle = (shoulderValue * 360 / 4096.0f) - 0;
        if (angle < 0) angle += 360;

        return angle;
    }

    /**
     * <p>The angle of the arm's elbow joint</p>
     * <p>Ranges from zero to 360</p>
     */
    public float getElbowAngle() {
        var angle = (elbowValue * 360 / 4096.0f) - 149;
        if (angle < 0) angle += 360;

        return angle;
    }

    /**
     * <p>The angle of the arm's wrist joint</p>
     * <p>Ranges from zero to 360</p>
     */
    public float getWristAngle() {
        var angle = (wristValue * 360 / 4096.0f) - 37;
        if (angle < 0) angle += 360;

        return angle;
    }

    /**
     * <p>The velocity of the arm's shoulder joint</p>
     */
    public float getShoulderVelocity() {
        return shoulderVelocity;
    }

    /**
     * <p>The velocity of the arm's elbow joint</p>
     */
    public float getElbowVelocity() {
        return elbowVelocity;
    }

    /**
     * <p>The velocity of the arm's wrist joint</p>
     */
    public float getWristVelocity() {
        return wristVelocity;
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

    /*public enum Command {

        @Deprecated //unimplemented
        SET_MODE_COLOR_TRACKING((byte) 0);

        private byte id;

        Command(byte id) {
            this.id = id;
        }
    }*/
}
