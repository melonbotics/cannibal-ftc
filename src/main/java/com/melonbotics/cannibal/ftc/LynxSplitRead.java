package com.melonbotics.cannibal.ftc;

import com.qualcomm.hardware.lynx.LynxController;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.lynx.LynxModuleIntf;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.Supplier;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadStatusQueryCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadStatusQueryResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cWriteReadMultipleBytesCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxNack;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import java.lang.reflect.Field;
import java.lang.reflect.Method;

/**
 * Split register reads on a REV hub I²C port, built from the hub's own commands: start a
 * write-read and return once the hub acknowledges it, then later collect the result with
 * one status query. Every command still waits for its reply, exactly as the SDK's own I²C
 * reads do, so this behaves the same with an Expansion Hub on RS485.
 *
 * <p>The hub keeps one pending result per port until it is collected or another operation
 * starts on that port, so nothing else may use the port in between. The module-wide I²C
 * lock is held only while each phase runs; devices on other ports are free meanwhile.
 *
 * <p>The SDK does not expose a device's module or bus, so they are read by reflection. If
 * that fails, or the hub firmware lacks the combined write-read command, {@link #of}
 * returns null and the BRIDGE reads normally.
 */
final class LynxSplitRead {
    private static final long BUSY_LIMIT_NANOS = 100_000_000L;
    private static final Method MODULE = method();
    private static final Field BUS = field();
    private final I2cDeviceSynchSimple device;
    private final int bus;

    private LynxSplitRead(I2cDeviceSynchSimple device, int bus) { this.device = device; this.bus = bus; }

    static LynxSplitRead of(I2cDeviceSynchSimple device) {
        if (MODULE == null || BUS == null || !(device instanceof LynxI2cDeviceSynch)) return null;
        try {
            LynxModuleIntf module = module(device);
            if (module == null || !module.isCommandSupported(LynxI2cWriteReadMultipleBytesCommand.class)) return null;
            return new LynxSplitRead(device, BUS.getInt(device));
        } catch (ReflectiveOperationException | RuntimeException unavailable) {
            return null;
        }
    }

    /** Starts the read. False when the hub refused it; the caller then reads normally. */
    boolean start(final int register, final int length) {
        try {
            final LynxModuleIntf module = module(device);
            return module.acquireI2cLockWhile(new Supplier<Boolean>() {
                @Override public Boolean get() throws InterruptedException {
                    long deadline = System.nanoTime() + BUSY_LIMIT_NANOS;
                    for (;;) {
                        try {
                            new LynxI2cWriteReadMultipleBytesCommand(module, bus, device.getI2cAddress(), register, length).send();
                            return true;
                        } catch (LynxNackException nack) {
                            if (!busy(nack) || System.nanoTime() > deadline) return false;
                        }
                    }
                }
            });
        } catch (InterruptedException interrupted) {
            Thread.currentThread().interrupt();
            return false;
        } catch (RobotCoreException | LynxNackException | ReflectiveOperationException | RuntimeException refused) {
            return false;
        }
    }

    /**
     * Collects the read. Null when the hub holds no result (lost, or never started);
     * throws when the read failed on the bus.
     */
    byte[] finish(final int length) {
        try {
            final LynxModuleIntf module = module(device);
            return module.acquireI2cLockWhile(new Supplier<byte[]>() {
                @Override public byte[] get() throws InterruptedException {
                    long deadline = System.nanoTime() + BUSY_LIMIT_NANOS;
                    for (;;) {
                        try {
                            LynxI2cReadStatusQueryResponse response = new LynxI2cReadStatusQueryCommand(module, bus, length).sendReceive();
                            if (!response.isStatusOk()) throw new IllegalStateException(describe(response));
                            return response.getBytes();
                        } catch (LynxNackException nack) {
                            if (reason(nack) == LynxNack.StandardReasonCode.I2C_NO_RESULTS_PENDING) return null;
                            if (!busy(nack)) throw new IllegalStateException("I2C read refused: " + reason(nack));
                            if (System.nanoTime() > deadline) throw new IllegalStateException("I2C read did not finish");
                        }
                    }
                }
            });
        } catch (InterruptedException interrupted) {
            Thread.currentThread().interrupt();
            throw new IllegalStateException("I2C read interrupted");
        } catch (RobotCoreException | LynxNackException | ReflectiveOperationException failure) {
            throw new IllegalStateException("I2C read failed: " + failure.getMessage());
        }
    }

    private static boolean busy(LynxNackException nack) {
        LynxNack.StandardReasonCode code = reason(nack);
        return code == LynxNack.StandardReasonCode.I2C_MASTER_BUSY || code == LynxNack.StandardReasonCode.I2C_OPERATION_IN_PROGRESS;
    }

    private static LynxNack.StandardReasonCode reason(LynxNackException nack) {
        return nack.getNack().getNackReasonCodeAsEnum();
    }

    private static String describe(LynxI2cReadStatusQueryResponse response) {
        if (!response.isAddressAcknowledged()) return "I2C address not acknowledged";
        if (!response.isDataAcknowledged()) return "I2C data not acknowledged";
        if (response.isArbitrationLost()) return "I2C arbitration lost";
        if (response.isClockTimeout()) return "I2C clock timeout";
        return String.format("I2C status 0x%02x", response.getI2cStatus() & 0xff);
    }

    /** The module can change between real and pretend as the hub arms, so look it up each time. */
    private static LynxModuleIntf module(I2cDeviceSynchSimple device) throws ReflectiveOperationException {
        return (LynxModuleIntf) MODULE.invoke(device);
    }

    private static Method method() {
        try {
            Method method = LynxController.class.getDeclaredMethod("getModule");
            method.setAccessible(true);
            return method;
        } catch (ReflectiveOperationException | RuntimeException unavailable) {
            return null;
        }
    }

    private static Field field() {
        try {
            Field field = LynxI2cDeviceSynch.class.getDeclaredField("bus");
            field.setAccessible(true);
            return field;
        } catch (ReflectiveOperationException | RuntimeException unavailable) {
            return null;
        }
    }
}
