package com.melonbotics.cannibal;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.List;

/** Codec for the BRIDGE I²C interface, revision 2. No SDK dependencies. */
final class BridgeWire {
    static final int INFO = 0, COMMAND = 1, SNAPSHOT = 0x20, CONTINUATION = 0x21, METADATA = 0x22, HEALTH = 0x23, DIRECTORY = 0x24;
    static final int POSITION = 1, VELOCITY = 2, POSITION_EX = 4, VELOCITY_EX = 8, ABSOLUTE = 16;
    static final int COMPACT = POSITION | VELOCITY, EXTENDED = POSITION_EX | VELOCITY_EX;
    static final int LOCALISER = POSITION | VELOCITY;
    /** Localiser bit 2: field-frame velocity as three i16, mm/s, mm/s and mrad/s. */
    static final int COMPACT_TWIST = 4;
    static final int MAX_VIEWS = 32, MAX_SNAPSHOT = 1024;
    /** A UID view's bus when the BRIDGE may find the device on either CAN bus. */
    static final int EITHER_BUS = 2;
    /** Interface revision: identity byte 15 and the first byte the selection ID covers. */
    static final int REVISION = 2;
    static ByteBuffer buffer(int length) { return ByteBuffer.allocate(length).order(ByteOrder.LITTLE_ENDIAN); }
    static ByteBuffer wrap(byte[] bytes) { return ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN); }
    static long unsigned(int value) { return value & 0xffffffffL; }
    static void require(boolean condition, String why) { if (!condition) throw new IllegalArgumentException(why); }
    static int crc16(byte[] bytes, int length) { return crc16(0xffff, bytes, 0, length); }
    /** CRC-16/CCITT-FALSE register after bytes[offset, offset+length), starting from crc. */
    static int crc16(int crc, byte[] bytes, int offset, int length) {
        for (int i = offset; i < offset + length; i++) { crc ^= (bytes[i] & 255) << 8;
            for (int bit = 0; bit < 8; bit++) crc = ((crc << 1) ^ ((crc & 0x8000) != 0 ? 0x1021 : 0)) & 0xffff;
        }
        return crc;
    }
    /** The selection ID: a check over the revision, the view count and every VIEW as sent. */
    static int selectionId(List<Row> rows) {
        int crc = crc16(0xffff, new byte[] {(byte)REVISION, (byte)rows.size()}, 0, 2);
        for (Row row : rows) { byte[] view = row.encode(); crc = crc16(crc, view, 0, view.length); }
        return crc;
    }
    /** A UID48 as the hardware tools show it: its six opaque bytes in order, as XXXX-XXXX-XXXX. */
    static String uidText(long uid) {
        StringBuilder text = new StringBuilder();
        for (int i = 0; i < 6; i++) { if (i == 2 || i == 4) text.append('-'); text.append(String.format("%02x", (uid >> (8 * i)) & 255)); }
        return text.toString();
    }
    static int headerLength(int views) { return 2 + (views + 7) / 8; }
    static int snapshotLength(List<Row> rows) { int length = headerLength(rows.size()) + 2; for (Row row : rows) length += row.length(); return length; }
    static void checked(byte[] bytes, int length) {
        require(bytes != null && bytes.length == length && length >= 2, "incorrect read length");
        require((wrap(bytes).getShort(length - 2) & 65535) == crc16(bytes, length - 2), "I2C checksum mismatch");
    }
    static final class Info {
        final int maxViews, maxSnapshot, localPorts, remoteSlots, buses, address;
        final long capabilities, uptimeUs;
        Info(byte[] bytes) {
            checked(bytes, 26); ByteBuffer b = wrap(bytes);
            require(b.getInt() == 0x31524243, "unsupported BRIDGE identity");
            capabilities = unsigned(b.getInt()); maxViews = b.get() & 255; localPorts = b.get() & 255;
            remoteSlots = b.get() & 255; buses = b.get() & 255; maxSnapshot = b.getShort() & 65535;
            address = b.get() & 255; int revision = b.get() & 255; uptimeUs = b.getLong();
            require(revision >= REVISION, "BRIDGE firmware predates this CANnibal FTC library; update the BRIDGE firmware");
            require(revision == REVISION, "BRIDGE firmware is newer than this CANnibal FTC library; update the library");
            require((capabilities & ~7L) == 0 && maxViews > 0 && maxViews <= MAX_VIEWS && localPorts <= 6 && remoteSlots <= 24
                && buses <= 2 && maxSnapshot >= 34 && maxSnapshot <= MAX_SNAPSHOT && address >= 8 && address <= 0x77, "unsupported capabilities");
        }
    }
    static final class Row {
        /** The view index in the installed selection, or -1 while this row is not selected. */
        int index;
        final int kind, bus, deviceType, manufacturer;
        /** A UID, local port or zero; a named row learns its UID from the directory. */
        long identity;
        int fields;
        long cpr;
        /** The nickname a named row resolves, else null. */
        final String name;
        /** Whether the row's source is known: always, except for a name not yet found. */
        boolean resolved;
        /** Why a named row is unresolved, for warnings and getProblem(). */
        String problem;
        Row(int index, int kind, int bus, int deviceType, int manufacturer, long identity, int fields) {
            this(index, kind, bus, deviceType, manufacturer, identity, fields, null);
        }
        Row(int index, int kind, int bus, int deviceType, int manufacturer, long identity, int fields, String name) {
            this.index = index; this.kind = kind; this.bus = bus; this.deviceType = deviceType; this.manufacturer = manufacturer;
            this.identity = identity; this.fields = fields; this.name = name; this.resolved = name == null;
            admitted();
        }
        /** How warnings name this view's source. */
        String label() {
            switch (kind) {
                case 0: return "Encoder port " + identity;
                case 1: return name == null ? "Encoder " + uidText(identity) : resolved ? "Encoder " + name + " (" + uidText(identity) + ")" : "Encoder " + name;
                default: return "Localiser";
            }
        }
        boolean sameSource(int kind, int bus, int deviceType, int manufacturer, long identity, String name) {
            return this.kind == kind && this.bus == bus && this.deviceType == deviceType && this.manufacturer == manufacturer
                && (name == null ? this.name == null && this.identity == identity : name.equals(this.name));
        }
        void admitted() {
            require(index >= -1 && index < MAX_VIEWS && fields != 0, "invalid view");
            if (kind == 0) require(identity >= 0 && identity < 6 && bus == 0 && deviceType == 0 && manufacturer == 0 && (fields & ~15) == 0, "invalid local encoder view");
            else if (kind == 1) require(bus >= 0 && bus <= EITHER_BUS && deviceType > 0 && deviceType <= 31 && manufacturer > 0 && manufacturer <= 255 && identity >= 0 && identity < (1L << 48) && (fields & ~31) == 0, "invalid CAN encoder view");
            else if (kind == 3) require(identity == 0 && bus == 0 && deviceType == 0 && manufacturer == 0 && (fields & ~7) == 0, "invalid localiser view");
            else throw new IllegalArgumentException("unknown source kind");
        }
        byte[] encode() { admitted(); return buffer(16).put((byte)index).put((byte)kind).putShort((short)fields).put((byte)bus).put((byte)deviceType).put((byte)manufacturer).put((byte)0).putLong(identity).array(); }
        int length() { int[] widths = kind == 3 ? new int[] {12,12,6} : new int[] {4,2,8,4,2}; int size = 0;
            for (int bit = 0; bit < widths.length; bit++) if ((fields & (1 << bit)) != 0) size += widths[bit]; return size; }
        void metadata(byte[] bytes, int selectionId) {
            checked(bytes, 18); ByteBuffer b = wrap(bytes);
            require((b.get() & 255) == index && (b.get() & 255) == kind && (b.getShort() & 65535) == fields, "view metadata differs from selection");
            long counts = unsigned(b.getInt()); require((b.getShort() & 65535) == selectionId, "metadata selection changed");
            for (int i = 0; i < 6; i++) require(b.get() == 0, "reserved metadata byte");
            require(kind == 3 ? counts == 0 : counts > 0, "missing encoder units"); cpr = counts;
        }
    }
    static final class Sample {
        final int fields;
        final long position, cpr;
        final double revolutionsPerSecond, countsPerSecond;
        final int absolute;
        final boolean valid;
        final CannibalLocaliser.Pose pose, velocity;
        Sample(ByteBuffer b, Row row, boolean valid) {
            fields = row.fields; cpr = row.cpr; long count = 0; double rps = Double.NaN, cps = Double.NaN; int abs = 0;
            CannibalLocaliser.Pose p = CannibalLocaliser.Pose.ZERO, v = CannibalLocaliser.Pose.ZERO;
            if (row.kind == 3) {
                if ((fields & POSITION) != 0) p = new CannibalLocaliser.Pose(b.getInt() / 1e6, b.getInt() / 1e6, b.getInt() / 1e6);
                if ((fields & VELOCITY) != 0) v = new CannibalLocaliser.Pose(b.getInt() / 1e6, b.getInt() / 1e6, b.getInt() / 1e6);
                if ((fields & COMPACT_TWIST) != 0) {
                    CannibalLocaliser.Pose compact = new CannibalLocaliser.Pose(b.getShort() / 1e3, b.getShort() / 1e3, b.getShort() / 1e3);
                    if ((fields & VELOCITY) == 0) v = compact;
                }
            } else {
                if ((fields & POSITION) != 0) count = b.getInt();
                if ((fields & VELOCITY) != 0) { rps = b.getShort() / 128.0; cps = rps * row.cpr; }
                if ((fields & POSITION_EX) != 0) count = b.getLong();
                if ((fields & VELOCITY_EX) != 0) { cps = b.getInt(); rps = cps / row.cpr; }
                if ((fields & ABSOLUTE) != 0) abs = b.getShort() & 65535;
            }
            position = count; revolutionsPerSecond = rps; countsPerSecond = cps; absolute = abs; pose = p; velocity = v;
            this.valid = valid;
        }
    }
    /** Why a snapshot was refused: the driver reads health to tell a reset or lost selection from noise. */
    static final class SnapshotRejected extends IllegalArgumentException {
        private static final long serialVersionUID = 1L;
        final boolean empty;
        SnapshotRejected(String why, boolean empty) { super(why); this.empty = empty; }
    }
    /**
     * Admits a whole snapshot of this selection: its exact length, a nonzero sequence (so an
     * all-zero SDK error fill never passes), no unused valid bits, and a check seeded with the
     * selection ID, which the BRIDGE never sends.
     */
    static final class Snapshot {
        final int sequence, health;
        final long hostNanos;
        final Sample[] samples;
        /** The rows this snapshot was read for, in view order: a reselection never remaps it. */
        final List<Row> rows;
        Snapshot(byte[] bytes, List<Row> rows, int selectionId, long hostNanos) {
            this.rows = rows;
            int length = snapshotLength(rows);
            if (bytes == null || bytes.length != length) throw new SnapshotRejected("snapshot length", false);
            int check = crc16(crc16(0xffff, new byte[] {(byte)selectionId, (byte)(selectionId >> 8)}, 0, 2), bytes, 0, length - 2);
            boolean zero = true; for (byte value : bytes) zero &= value == 0;
            if (zero || bytes[0] == 0) throw new SnapshotRejected("no snapshot: the BRIDGE has no selection, or the read failed", true);
            if ((wrap(bytes).getShort(length - 2) & 65535) != check) throw new SnapshotRejected("snapshot checksum mismatch", false);
            ByteBuffer b = wrap(bytes); sequence = b.get() & 255; health = b.get() & 255;
            long valid = 0; int flags = headerLength(rows.size()) - 2;
            for (int i = 0; i < flags; i++) valid |= (long)(b.get() & 255) << (8 * i);
            long used = (1L << rows.size()) - 1; require((valid & ~used) == 0, "reserved snapshot valid bit");
            samples = new Sample[rows.size()];
            for (int i = 0; i < samples.length; i++) samples[i] = new Sample(b, rows.get(i), (valid & (1L << i)) != 0);
            require(b.position() == length - 2, "snapshot field length"); this.hostNanos = hostNanos;
        }
    }
    static final int HEALTH_BYTES = 94, HEALTH_EVENTS = 6, NO_VIEW = 255;
    static final int EVENT_BRIDGE_RESET = 1, EVENT_SOURCE_RESTARTED = 2, EVENT_SOURCE_MISSING = 3, EVENT_SOURCE_RETURNED = 4,
        EVENT_UID_UNRESOLVED = 5, EVENT_UID_DUPLICATE = 6, EVENT_NUMBER_CONFLICT = 7, EVENT_CAN_BUS_OFF = 8, EVENT_CAN_RECOVERED = 9,
        EVENT_SELECTION_REJECTED = 10, EVENT_SELECTION_INVALIDATED = 11, EVENT_IMU_CALIBRATING = 12, EVENT_IMU_CALIBRATED = 13,
        EVENT_COMMAND_APPLIED = 14, EVENT_IMU_CONFIG_FAULT = 15;
    static final int REJECT_MALFORMED = 1, REJECT_UNSUPPORTED = 2, REJECT_LOCAL_PORT = 3, REJECT_TOO_LARGE = 4, REJECT_MISMATCH = 5;
    /** One recent BRIDGE event; unknown kinds are kept and described generically. */
    static final class Event {
        final int sequence, kind, view, detail;
        final byte[] argument;
        Event(ByteBuffer b) {
            sequence = b.get() & 255; kind = b.get() & 255; view = b.get() & 255; detail = b.get() & 255;
            argument = new byte[6]; b.get(argument);
        }
        long token() { return unsigned(wrap(argument).getInt()); }
        long uid() { long value = 0; for (int i = 0; i < 6; i++) value |= (long)(argument[i] & 255) << (8 * i); return value; }
    }
    /** Register 0x23: state, counters and the recent events, newest first. */
    static final class Health {
        final int counter, latestEvent, directoryGeneration, resetCause, i2cBusErrors, rejectedWrites, routeLapses;
        final boolean selectionActive, imuCalibrating;
        final int selectionId;
        final long uptimeMs;
        final int[] canState = new int[2], canInterruptions = new int[2], suppliesMv = new int[4];
        final Event[] events;
        Health(byte[] bytes) {
            checked(bytes, HEALTH_BYTES); ByteBuffer b = wrap(bytes);
            counter = b.get() & 255; latestEvent = b.get() & 255; directoryGeneration = b.get() & 255;
            int flags = b.get() & 255; require((flags & ~3) == 0, "reserved health state");
            selectionActive = (flags & 1) != 0; imuCalibrating = (flags & 2) != 0; selectionId = b.getShort() & 65535;
            require(selectionActive || selectionId == 0, "selection ID without a selection");
            uptimeMs = unsigned(b.getInt()); resetCause = b.get() & 255;
            for (int i = 0; i < 2; i++) canState[i] = b.get() & 255;
            for (int i = 0; i < 4; i++) suppliesMv[i] = b.getShort() & 65535;
            for (int i = 0; i < 2; i++) canInterruptions[i] = b.getShort() & 65535;
            i2cBusErrors = b.getShort() & 65535; rejectedWrites = b.getShort() & 65535; routeLapses = b.getShort() & 65535;
            require(b.get() == 0, "reserved health byte");
            java.util.ArrayList<Event> recent = new java.util.ArrayList<>();
            for (int i = 0; i < HEALTH_EVENTS; i++) {
                Event event = new Event(b);
                if (event.sequence != 0) recent.add(event);
                else require(event.kind == 0 && event.view == 0 && event.detail == 0 && event.uid() == 0, "empty health event slot");
            }
            events = recent.toArray(new Event[0]);
        }
    }
    static final int DIRECTORY_BYTES = 23, DIRECTORY_HEADER = 255;
    static final int DEVICE_HEARD = 1, DEVICE_BOOTLOADER = 2, DEVICE_NO_NICKNAME = 4, DEVICE_CONFLICT = 8;
    static final String NICKNAME_ALPHABET = "abcdefghijklmnopqrstuvwxyz0123456789-";
    /** The largest valid encoded nickname: the sum of 37^1 through 37^12. */
    static final long NICKNAME_MAX = 6_765_811_783_780_036_260L;
    /** Register 0x24: the header at index 255, else one device. */
    static final class DirectoryRecord {
        final int index, generation, count, deviceType, manufacturer, route, number, flags;
        final long uid, nickname;
        DirectoryRecord(byte[] bytes) {
            checked(bytes, DIRECTORY_BYTES); ByteBuffer b = wrap(bytes);
            index = b.get() & 255; generation = b.get() & 255;
            if (index == DIRECTORY_HEADER) {
                count = b.get() & 255; for (int i = 3; i < 21; i++) require(b.get() == 0, "reserved directory header byte");
                uid = 0; nickname = 0; deviceType = manufacturer = route = number = flags = 0;
                return;
            }
            count = 0; long id = 0; for (int i = 0; i < 6; i++) id |= (long)(b.get() & 255) << (8 * i); uid = id;
            deviceType = b.get() & 255; manufacturer = b.get() & 255; route = b.get() & 255; number = b.get() & 255;
            nickname = b.getLong(); flags = b.get() & 255;
            require(route <= 2 && number <= 62 && deviceType > 0 && deviceType <= 31 && manufacturer > 0 && (flags & ~15) == 0, "invalid directory entry");
            require(Long.compareUnsigned(nickname, NICKNAME_MAX) <= 0, "invalid nickname");
            require((flags & DEVICE_NO_NICKNAME) == 0 || nickname == 0, "nickname on an unnamed entry");
        }
    }
    /** device-nickname-v1: bijective base 37, empty is zero. */
    static String nicknameText(long value) {
        StringBuilder text = new StringBuilder();
        while (value != 0) {
            long digit = Long.remainderUnsigned(value - 1, 37);
            text.append(NICKNAME_ALPHABET.charAt((int)digit));
            value = Long.divideUnsigned(value - 1, 37);
        }
        return text.reverse().toString();
    }
    static boolean validNickname(String text) { return text != null && text.matches("[a-z0-9-]{1,12}"); }
    static byte[] begin(int count) { return buffer(2).put((byte)1).put((byte)count).array(); }
    static byte[] view(Row row) { return buffer(17).put((byte)2).put(row.encode()).array(); }
    static byte[] commit(int selectionId) { return buffer(3).put((byte)3).putShort((short)selectionId).array(); }
    static byte[] offset(int offset) { return buffer(2).putShort((short)offset).array(); }
    static byte[] runtime(int opcode, int selectionId, long token, int[] values) {
        require(selectionId >= 0 && selectionId <= 0xffff && token > 0 && token <= 0xffffffffL, "invalid runtime correlation");
        require((opcode == 4 && values.length == 3) || (opcode == 5 && values.length == 0) || (opcode == 6 && values.length == 1), "invalid runtime command");
        ByteBuffer b = buffer(opcode == 4 ? 19 : opcode == 6 ? 8 : 7).put((byte)opcode).putShort((short)selectionId).putInt((int)token);
        if (opcode == 4) for (int value : values) b.putInt(value);
        if (opcode == 6) { require(values[0] >= 0 && values[0] < MAX_VIEWS, "invalid encoder index"); b.put((byte)values[0]); }
        return b.array();
    }
    private BridgeWire() { }
}
