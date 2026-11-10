package com.melonbotics.cannibal;

/** One CANnibal device the BRIDGE has heard, as its directory listed it. */
public final class BridgeDevice {
    /** The device's UID48 as XXXX-XXXX-XXXX, the form Hardware Manager shows. */
    public final String uid;
    /** Its own nickname; empty when it has none, null when none has been heard yet. */
    public final String name;
    /** FIRST CAN product type and manufacturer: a Nano Encoder is 7 and 8. */
    public final int deviceType, manufacturer;
    /** "CAN1", "CAN2", or "local" for the BRIDGE itself. */
    public final String bus;
    public final int deviceNumber;
    /** Heard within the last four Presence periods. */
    public final boolean heardRecently;
    /** Running its bootloader rather than its application. */
    public final boolean inBootloader;
    /** Another UID claims the same device number on its bus. */
    public final boolean numberConflict;
    final long uidValue;

    BridgeDevice(BridgeWire.DirectoryRecord record) {
        uidValue = record.uid; uid = BridgeWire.uidText(record.uid);
        name = (record.flags & BridgeWire.DEVICE_NO_NICKNAME) != 0 ? null : BridgeWire.nicknameText(record.nickname);
        deviceType = record.deviceType; manufacturer = record.manufacturer;
        bus = record.route == 2 ? "local" : "CAN" + (record.route + 1); deviceNumber = record.number;
        heardRecently = (record.flags & BridgeWire.DEVICE_HEARD) != 0; inBootloader = (record.flags & BridgeWire.DEVICE_BOOTLOADER) != 0;
        numberConflict = (record.flags & BridgeWire.DEVICE_CONFLICT) != 0;
    }
    public boolean isEncoder() { return deviceType == 7 && manufacturer == 8; }
    @Override public String toString() {
        return (name == null || name.isEmpty() ? "(unnamed)" : name) + " " + uid + " type " + deviceType + " on " + bus + " #" + deviceNumber
            + (heardRecently ? "" : ", silent") + (inBootloader ? ", bootloader" : "") + (numberConflict ? ", number conflict" : "");
    }
}
