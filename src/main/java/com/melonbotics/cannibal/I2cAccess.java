package com.melonbotics.cannibal;

/** Complete register transactions; the FTC and native-fixture adapters share this seam. */
public interface I2cAccess {
    byte[] read(int register, int length);
    void write(int register, byte[] payload);

    /**
     * Optional split read, used by {@link BridgeClient#prefetch()}: begins a register read
     * and returns once the transport has accepted it, without waiting for the data. Returns
     * false when this transport cannot split a read or the start was refused; the caller
     * then reads normally.
     */
    default boolean startRead(int register, int length) { return false; }

    /**
     * Collects the read begun by {@link #startRead}. Returns null when the transport holds
     * no result, because it was lost or never started, so the caller reads normally
     * instead. Throws when the read itself failed, for example when the target did not
     * answer.
     */
    default byte[] finishRead(int register, int length) { return null; }
}
