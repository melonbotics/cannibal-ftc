package com.melonbotics.cannibal;

import java.util.Collections;
import java.util.List;

/**
 * The BRIDGE's health when it was last read: what its warnings were about, and the counts
 * behind them. The driver reads it only when the snapshot says something changed, or after
 * a failed read, so it can be up to a loop or so old.
 */
public final class BridgeHealth {
    /** Time since the BRIDGE started. */
    public final long uptimeMillis;
    /** Why it last started: "power-on", "brownout", "reset pin", "software", "watchdog" or "unknown". */
    public final String resetCause;
    /** Input supplies in volts; NaN when unavailable. */
    public final double usbVolts, hubVolts, can1Volts, can2Volts;
    /** "error active", "warning", "passive", "bus-off", "stopped", "sleeping" or "unknown". */
    public final String can1State, can2State;
    /** Received frames lost, or bus-off, on each bus since the BRIDGE started. */
    public final int can1Interruptions, can2Interruptions;
    public final int i2cBusErrors, rejectedWrites;
    /** Identity confirmations a CAN device missed while its frames continued. */
    public final int routeLapses;
    public final boolean imuCalibrating;
    /** The BRIDGE's six newest events, oldest first, in words. */
    public final List<String> recentEvents;

    BridgeHealth(BridgeWire.Health report, List<String> recent) {
        uptimeMillis = report.uptimeMs; resetCause = resetCause(report.resetCause);
        usbVolts = volts(report.suppliesMv[0]); hubVolts = volts(report.suppliesMv[1]);
        can1Volts = volts(report.suppliesMv[2]); can2Volts = volts(report.suppliesMv[3]);
        can1State = canState(report.canState[0]); can2State = canState(report.canState[1]);
        can1Interruptions = report.canInterruptions[0]; can2Interruptions = report.canInterruptions[1];
        i2cBusErrors = report.i2cBusErrors; rejectedWrites = report.rejectedWrites; routeLapses = report.routeLapses;
        imuCalibrating = report.imuCalibrating; recentEvents = Collections.unmodifiableList(recent);
    }
    static String resetCause(int cause) {
        switch (cause) {
            case 1: return "power-on";
            case 2: return "brownout";
            case 3: return "reset pin";
            case 4: return "software";
            case 5: return "watchdog";
            default: return "unknown";
        }
    }
    private static String canState(int state) {
        switch (state) {
            case 0: return "error active";
            case 1: return "warning";
            case 2: return "passive";
            case 3: return "bus-off";
            case 4: return "stopped";
            case 5: return "sleeping";
            default: return "unknown";
        }
    }
    private static double volts(int millivolts) { return millivolts == 65535 ? Double.NaN : millivolts / 1000.0; }
    @Override public String toString() {
        return String.format("up %.1f s (reset: %s), CAN1 %s, CAN2 %s, route lapses %d", uptimeMillis / 1000.0, resetCause, can1State, can2State, routeLapses);
    }
    static String configurationFault(BridgeWire.Event event) {
        return String.format(java.util.Locale.ROOT,
            "BRIDGE IMU %d not configured (register 0x%02X wrote 0x%02X, read 0x%02X%s); excluded from heading",
            event.detail + 1, event.argument[0] & 255, event.argument[1] & 255, event.argument[2] & 255,
            event.argument[3] != 0 ? "; transport failed" : "");
    }
}
