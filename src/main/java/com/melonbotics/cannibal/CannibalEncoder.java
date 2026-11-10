package com.melonbotics.cannibal;

/**
 * One encoder on the BRIDGE: a CAN encoder found by its nickname or UID, or a local
 * quadrature port. Its fields are declared when it is created; getters read the cache from
 * the last update() and cause no I²C traffic.
 */
public final class CannibalEncoder {
    /** What a handle reads. With none declared it reads POSITION and VELOCITY, six bytes. */
    public enum Field {
        /** Accumulated counts, modulo 2^32: 4 bytes. */
        POSITION(BridgeWire.POSITION),
        /** Velocity in 1/128 revolutions per second, up to ±15,360 RPM: 2 bytes. */
        VELOCITY(BridgeWire.VELOCITY),
        /** Accumulated counts as a 64-bit integer: 8 bytes. */
        POSITION_EXTENDED(BridgeWire.POSITION_EX),
        /** Velocity in whole counts per second: 4 bytes. */
        VELOCITY_EXTENDED(BridgeWire.VELOCITY_EX),
        /** The angle within one revolution, CAN encoders only: 2 bytes. A swerve azimuth needs only this and VELOCITY. */
        ABSOLUTE(BridgeWire.ABSOLUTE);
        final int bit;
        Field(int bit) { this.bit = bit; }
        static int mask(Field[] fields) {
            if (fields == null || fields.length == 0) return BridgeWire.COMPACT;
            int mask = 0; for (Field field : fields) { if (field == null) throw new NullPointerException("field"); mask |= field.bit; }
            return mask;
        }
    }
    private static final int ANY_POSITION = BridgeWire.POSITION | BridgeWire.POSITION_EX;
    private static final int ANY_VELOCITY = BridgeWire.VELOCITY | BridgeWire.VELOCITY_EX;
    final BridgeClient bridge; final BridgeWire.Row row; final int declared;
    CannibalEncoder(BridgeClient bridge, BridgeWire.Row row, int declared) { this.bridge = bridge; this.row = row; this.declared = declared; }
    private BridgeWire.Sample sample(int needs, String getter, String fields) {
        if ((declared & needs) == 0)
            throw new IllegalStateException(row.label() + " was created without " + fields + "; declare it when creating the handle to use " + getter + "()");
        return bridge.sample(row);
    }
    /** Accumulated counts since the BRIDGE started observing, less zero(); modulo 2^32 unless POSITION_EXTENDED was declared. */
    public long getPosition() { BridgeWire.Sample s = sample(ANY_POSITION, "getPosition", "POSITION or POSITION_EXTENDED"); return s == null ? 0 : s.position; }
    public double getPositionRadians() { BridgeWire.Sample s = sample(ANY_POSITION, "getPositionRadians", "POSITION or POSITION_EXTENDED"); return s == null || s.cpr == 0 ? Double.NaN : s.position * (2 * Math.PI / s.cpr); }
    /** Counts per second. */
    public double getVelocity() { BridgeWire.Sample s = sample(ANY_VELOCITY, "getVelocity", "VELOCITY or VELOCITY_EXTENDED"); return s == null ? Double.NaN : s.countsPerSecond; }
    public double getVelocityRadiansPerSecond() { BridgeWire.Sample s = sample(ANY_VELOCITY, "getVelocityRadiansPerSecond", "VELOCITY or VELOCITY_EXTENDED"); return s == null ? Double.NaN : s.revolutionsPerSecond * 2 * Math.PI; }
    /** The angle within one revolution in native counts (Nano Encoder: 0 to 16383). */
    public int getAbsoluteCounts() { BridgeWire.Sample s = sample(BridgeWire.ABSOLUTE, "getAbsoluteCounts", "ABSOLUTE"); return s == null ? 0 : s.absolute; }
    public double getAbsoluteAngleRadians() { BridgeWire.Sample s = sample(BridgeWire.ABSOLUTE, "getAbsoluteAngleRadians", "ABSOLUTE"); return s == null || s.cpr == 0 ? Double.NaN : s.absolute * (2 * Math.PI / s.cpr); }
    public long getCountsPerRevolution() { BridgeWire.Sample s = bridge.sample(row); return s == null ? 0 : s.cpr; }
    /** A usable number: the source is present, fresh and in range, and the last bridge refresh is recent. */
    public boolean isValid() { BridgeWire.Sample s = bridge.sample(row); return bridge.hasFreshRead() && s != null && s.valid; }
    /** The nickname this handle was created with, or null. */
    public String getName() { return row.name; }
    /** The CAN encoder's UID as XXXX-XXXX-XXXX once known; null for a local port or a name not yet found. */
    public String getUid() { return row.kind == 1 && row.resolved ? BridgeWire.uidText(row.identity) : null; }
    /** Why a named handle cannot select its encoder yet, such as a missing or duplicated name; null otherwise. */
    public String getProblem() { return row.problem; }
    /**
     * Makes getPosition() read zero from now on. The absolute angle is unaffected, nothing is
     * written to the encoder, and odometry is undisturbed. Queued for update(); UNKNOWN means
     * a reset prevented acknowledgement.
     */
    public BridgeClient.Command zero() { return bridge.enqueueZero(row); }
    @Override public String toString() { return row.label(); }
}
