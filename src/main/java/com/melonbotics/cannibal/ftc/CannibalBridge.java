package com.melonbotics.cannibal.ftc;

import com.melonbotics.cannibal.*;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.RobotLog;

/** Select "CANnibal Bridge" on a Control Hub I²C port, then obtain it from hardwareMap. */
@I2cDeviceType
@DeviceProperties(name="CANnibal Bridge", xmlTag="CANnibalBridge", description="CAN and quadrature encoders with onboard localisation")
public final class CannibalBridge extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> {
    /** REV hub I²C port clock. The SDK resets every port to 100 kHz when an OpMode starts. */
    public enum I2cSpeed { STANDARD_100K, FAST_400K }
    /** 400 kHz roughly halves a snapshot read; use 100 kHz only when a 100 kHz-only device shares the port. */
    private static final I2cSpeed DEFAULT_SPEED = I2cSpeed.FAST_400K;
    private BridgeClient client;
    private I2cSpeed speed = DEFAULT_SPEED;
    public CannibalBridge(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        deviceClient.setI2cAddress(I2cAddr.create7bit(0x42));
        client = createClient(); registerArmingStateCallback(false); engage();
    }
    private BridgeClient createClient() {
        BridgeClient created = new BridgeClient(new I2cAccess() {
            // Looked up on first use rather than here: while the hub is still arming the
            // SDK may hand out a stand-in module that reports no command support.
            private LynxSplitRead split;
            public byte[] read(int register, int length) { return deviceClient.read(register,length); }
            // A hub port runs one transaction at a time and the SDK retries a start that
            // finds it busy, so the next read still follows this write. Waiting for the
            // write's own completion status would only add a round trip.
            public void write(int register, byte[] payload) { deviceClient.write(register,payload,I2cWaitControl.ATOMIC); }
            public boolean startRead(int register, int length) {
                if (split == null) split = LynxSplitRead.of(deviceClient);
                return split != null && split.start(register, length);
            }
            public byte[] finishRead(int register, int length) { return split == null ? null : split.finish(length); }
        });
        // BRIDGE health events, such as a restarted encoder, become RobotLog warnings.
        created.setWarningListener(message -> RobotLog.w(message));
        return created;
    }
    @Override protected synchronized boolean doInitialize() { applySpeed(); client.reconnect(); return client.probe(); }
    @Override public Manufacturer getManufacturer() { return Manufacturer.Other; }
    @Override public String getDeviceName() { return "CANnibal Bridge"; }
    @Override public synchronized void resetDeviceConfigurationForOpMode() {
        super.resetDeviceConfigurationForOpMode(); speed = DEFAULT_SPEED; client = createClient();
    }
    public synchronized void setI2cAddress(int sevenBitAddress) {
        if (sevenBitAddress < 8 || sevenBitAddress > 0x77) throw new IllegalArgumentException("invalid I2C address");
        deviceClient.setI2cAddress(I2cAddr.create7bit(sevenBitAddress)); client.reconnect();
    }
    /** Sets this port's I²C clock for the current OpMode. Every device on the port must support it. */
    public synchronized void setI2cSpeed(I2cSpeed speed) {
        if (speed == null) throw new NullPointerException("speed");
        this.speed = speed; applySpeed();
    }
    public synchronized I2cSpeed getI2cSpeed() { return speed; }
    private void applySpeed() {
        if (deviceClient instanceof LynxI2cDeviceSynch) ((LynxI2cDeviceSynch)deviceClient).setBusSpeed(speed == I2cSpeed.FAST_400K
            ? LynxI2cDeviceSynch.BusSpeed.FAST_400K : LynxI2cDeviceSynch.BusSpeed.STANDARD_100K);
    }
    public BridgeClient client() { return client; }
    /**
     * Collects the latest snapshot: the one prefetch() started, or a fresh read. Call once
     * per loop before reading any handle.
     */
    public boolean update() { return client.update(); }
    /**
     * Optional. Starts the next update()'s read and returns in about a millisecond, so the
     * hub clocks the snapshot while your loop sends motor and servo commands. Call it after
     * reading your sensors, before your writes. Uses only the hub's normal acknowledged
     * commands, so it is safe with an Expansion Hub on RS485. Nothing else may share the
     * BRIDGE's I²C port.
     */
    public boolean prefetch() { return client.prefetch(); }
    /** A CAN encoder by its own nickname; see {@link BridgeClient#encoderByName}. */
    public CannibalEncoder encoderByName(String name, CannibalEncoder.Field... fields) { return client.encoderByName(name, fields); }
    /** A CAN encoder by UID, XXXX-XXXX-XXXX or 12 hex digits, on either CAN bus. */
    public CannibalEncoder encoderByUid(String uid, CannibalEncoder.Field... fields) { return client.encoderByUid(uid, fields); }
    /** A quadrature encoder on a local port. */
    public CannibalEncoder localEncoder(int port, CannibalEncoder.Field... fields) { return client.localEncoder(port, fields); }
    public CannibalLocaliser getLocaliser() { return client.getLocaliser(); }
    public CannibalLocaliser getLocaliser(CannibalLocaliser.Velocity velocity) { return client.getLocaliser(velocity); }
    public BridgeClient.Command calibrateImu() { return client.calibrateImu(); }
    /** The BRIDGE's state when its health was last read, or null before then. */
    public BridgeHealth getHealth() { return client.getHealth(); }
    /** Every CANnibal device the BRIDGE has heard. One I²C read per device: not for every loop. */
    public java.util.List<BridgeDevice> readDirectory() { return client.readDirectory(); }
}
