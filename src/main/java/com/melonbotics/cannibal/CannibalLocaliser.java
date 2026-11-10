package com.melonbotics.cannibal;

/** Pose and field-frame velocity computed in BRIDGE firmware, never integrated in Java. */
public final class CannibalLocaliser {
    /**
     * How the field-frame velocity is read: FULL as micrometres and microradians per second
     * (12 bytes), COMPACT as millimetres and milliradians per second up to ±32.767 (6 bytes),
     * or NONE for pose alone.
     */
    public enum Velocity {
        FULL(BridgeWire.POSITION | BridgeWire.VELOCITY), COMPACT(BridgeWire.POSITION | BridgeWire.COMPACT_TWIST), NONE(BridgeWire.POSITION);
        final int fields;
        Velocity(int fields) { this.fields = fields; }
    }
    final BridgeClient bridge; final BridgeWire.Row row; final int required;
    CannibalLocaliser(BridgeClient bridge, BridgeWire.Row row, int required) { this.bridge = bridge; this.row = row; this.required = required; }
    public Pose getPose() { BridgeWire.Sample s = bridge.sample(row); return s == null ? Pose.ZERO : s.pose; }
    /** x/y are metres per second; heading is radians per second. Zero when velocity was not selected. */
    public Pose getVelocity() { BridgeWire.Sample s = bridge.sample(row); return s == null ? Pose.ZERO : s.velocity; }
    public boolean isValid() { BridgeWire.Sample s = bridge.sample(row); return bridge.hasFreshRead() && s != null && s.valid && (s.fields & required) == required; }
    /**
     * Queues a pose reset. A pose that is not finite, or beyond ±2147 m, comes back REFUSED
     * with a RobotLog warning rather than throwing, so a bad computed pose cannot stop a match.
     */
    public BridgeClient.Command setPose(double xMetres, double yMetres, double headingRadians) {
        if (!inRange(xMetres) || !inRange(yMetres) || !Double.isFinite(headingRadians))
            return bridge.refuse("setPose(" + xMetres + ", " + yMetres + ", " + headingRadians + ") is not a finite pose within ±2147 m");
        double wrapped = Math.atan2(Math.sin(headingRadians), Math.cos(headingRadians));
        return bridge.enqueue(4, fixed(xMetres), fixed(yMetres), fixed(wrapped));
    }
    private static boolean inRange(double metres) { double scaled = metres * 1e6; return Double.isFinite(scaled) && scaled >= Integer.MIN_VALUE && scaled <= Integer.MAX_VALUE; }
    private static int fixed(double value) { return (int)Math.round(value * 1e6); }
    public static final class Pose {
        static final Pose ZERO = new Pose(0,0,0);
        public final double x, y, heading;
        public Pose(double x, double y, double heading) { this.x = x; this.y = y; this.heading = heading; }
    }
}
