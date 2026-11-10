package com.melonbotics.cannibal;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.LongSupplier;

/**
 * Shared client. Factory methods declare data; update() owns all robot-runtime I²C
 * traffic. prefetch() is optional: it starts the next update()'s read early so the hub
 * clocks the snapshot while the loop does other work.
 */
public final class BridgeClient {
    private final I2cAccess access;
    private final LongSupplier clock;
    /** Every handle's source, in the order handles were created. */
    private final List<BridgeWire.Row> rows = new ArrayList<>();
    /** The rows in the installed selection, in view order: named rows join once found. */
    private List<BridgeWire.Row> selectedRows = new ArrayList<>();
    /** Directory generation when names were last resolved, and the newest health has reported. */
    private int resolvedGeneration = -1, reportedGeneration = -1, walkedGeneration = -1;
    private long lastDirectoryRead;
    /** While a name is missing, the directory is read again at least this often. */
    private static final long NAME_RETRY_NANOS = 1_000_000_000L;
    private final ArrayDeque<Command> commands = new ArrayDeque<>();
    private volatile BridgeWire.Snapshot snapshot;
    private volatile boolean healthy;
    private volatile String lastError = "no observation";
    private volatile long maximumCacheAgeNanos = 100_000_000L;
    private long nextToken;
    private int selectionId;
    private boolean selected;
    private BridgeWire.Info info;
    private int chunkSize = 100;
    private volatile UpdateTiming lastTiming;
    private int transactions;
    private volatile long maximumPrefetchAgeNanos = 50_000_000L;
    private boolean prefetchPending;
    private int prefetchLength;
    private long prefetchStarted;
    /** Last accepted snapshot sequence for the current selection, or -1 to accept any. */
    private int lastSequence = -1;
    /** Health counter of the last snapshot whose news was read, or -1 to read it now. */
    private int lastHealthCounter = -1;
    /** Newest event already handled; 0 treats every event as news, -1 before the first read. */
    private int lastEvent = -1;
    private long lastUptimeMs = -1, lastHealthNanos;
    /** Health news is read at most this often, so a flapping source costs one read per 100 ms, not one per loop. */
    static final long HEALTH_INTERVAL_NANOS = 100_000_000L;
    /** When health was last read, whatever the reason. */
    private long lastHealthRead;
    /** The last refused command's reason and when it was logged, so a loop repeating it logs once a second. */
    private String lastRefusal; private long lastRefusalNanos;
    private long lastReadWarningNanos; private long failedUpdates;
    private volatile BridgeHealth health;
    private volatile Consumer<String> warnings = message -> { };

    public BridgeClient(I2cAccess access) { this(access, System::nanoTime); }
    /** Clock injection for deterministic tests; values must be monotonic nanoseconds. */
    public BridgeClient(I2cAccess access, LongSupplier clock) {
        if (access == null || clock == null) throw new NullPointerException("transport and clock are required");
        this.access = access; this.clock = clock;
        lastHealthRead = clock.getAsLong() - HEALTH_INTERVAL_NANOS;
        lastReadWarningNanos = clock.getAsLong() - 1_000_000_000L;
    }
    public synchronized boolean probe() {
        try { info = new BridgeWire.Info(access.read(BridgeWire.INFO, 26)); return true; }
        catch (RuntimeException failure) { healthy = false; lastError = failure.getMessage(); return false; }
    }
    public synchronized void reconnect() { loseSelection(); healthy = false; info = null; lastEvent = -1; lastUptimeMs = -1; }
    public synchronized void setReadChunkSize(int bytes) {
        if (bytes < 32 || bytes > 100) throw new IllegalArgumentException("chunk size must be 32..100 bytes"); chunkSize = bytes;
    }
    public void setMaximumCacheAgeMillis(long milliseconds) {
        if (milliseconds < 1 || milliseconds > 60000) throw new IllegalArgumentException("cache age must be 1..60000 ms");
        maximumCacheAgeNanos = milliseconds * 1_000_000L;
    }
    /**
     * A prefetched snapshot older than this when update() collects it is dropped and read
     * again, so a stalled loop never acts on old numbers. Default 50 ms.
     */
    public void setMaximumPrefetchAgeMillis(long milliseconds) {
        if (milliseconds < 1 || milliseconds > 1000) throw new IllegalArgumentException("prefetch age must be 1..1000 ms");
        maximumPrefetchAgeNanos = milliseconds * 1_000_000L;
    }
    /**
     * Receives one line per BRIDGE health event worth a warning, such as
     * "CANnibal: Encoder 3a2c-8aec-75f0 restarted". The FTC device sends them to RobotLog.
     */
    public void setWarningListener(Consumer<String> listener) {
        if (listener == null) throw new NullPointerException("listener");
        warnings = listener;
    }
    /** The BRIDGE's state when its health was last read, or null before the first read. */
    public BridgeHealth getHealth() { return health; }
    /**
     * Reads the BRIDGE's device directory: itself, then every CANnibal device it has heard.
     * This costs one I²C read per device, so call it at setup or for telemetry, not every loop.
     * It never throws: the BRIDGE always lists itself first, so an empty list means the read
     * failed, and getLastError() says why.
     */
    public synchronized List<BridgeDevice> readDirectory() {
        try { return directory(); }
        catch (RuntimeException failure) { lastError = "directory: " + failure.getMessage(); return new ArrayList<>(); }
    }
    private List<BridgeDevice> directory() {
        prefetchPending = false;
        for (int attempt = 0; attempt < 3; attempt++) {
            List<BridgeDevice> devices = directoryOnce();
            if (devices != null) return devices;
        }
        throw new IllegalStateException("the BRIDGE directory kept changing while it was read");
    }
    private boolean waitingForNames() {
        for (BridgeWire.Row row : rows) if (!row.resolved) return true;
        return false;
    }
    /**
     * Finds each named row's encoder in the directory. One match pins the row to its UID and
     * reselects; none, or two or more, leave it waiting and invalid, with a warning naming them.
     */
    private void resolveNames() {
        if (!waitingForNames()) return;
        List<BridgeDevice> devices = directory();
        lastDirectoryRead = clock.getAsLong(); resolvedGeneration = walkedGeneration;
        boolean found = false;
        for (BridgeWire.Row row : rows) {
            if (row.resolved) continue;
            List<BridgeDevice> matches = new ArrayList<>();
            for (BridgeDevice device : devices) if (device.isEncoder() && row.name.equals(device.name)) matches.add(device);
            String problem;
            if (matches.size() == 1) {
                row.identity = matches.get(0).uidValue; row.resolved = true; row.problem = null; found = true;
                continue;
            } else if (matches.isEmpty()) {
                problem = "no encoder named " + row.name + " has been heard; its handle stays invalid until one appears";
            } else {
                StringBuilder uids = new StringBuilder();
                for (BridgeDevice match : matches) uids.append(uids.length() == 0 ? "" : ", ").append(match.uid);
                problem = "error: " + matches.size() + " encoders are named " + row.name + " (" + uids + "); rename all but one in Hardware Manager";
            }
            if (!problem.equals(row.problem)) warnings.accept("CANnibal: " + problem);
            row.problem = problem;
        }
        if (found) loseSelection();
    }
    /** One walk of register 0x24, or null when it changed or a record failed its check. */
    private List<BridgeDevice> directoryOnce() {
        write(BridgeWire.DIRECTORY, new byte[]{(byte)BridgeWire.DIRECTORY_HEADER});
        // Each read moves the BRIDGE to the next record.
        BridgeWire.DirectoryRecord header = new BridgeWire.DirectoryRecord(read(BridgeWire.DIRECTORY, BridgeWire.DIRECTORY_BYTES));
        BridgeWire.require(header.index == BridgeWire.DIRECTORY_HEADER, "directory header expected");
        walkedGeneration = header.generation;
        List<BridgeDevice> devices = new ArrayList<>();
        for (int index = 0; index < header.count; index++) {
            BridgeWire.DirectoryRecord record;
            try { record = new BridgeWire.DirectoryRecord(read(BridgeWire.DIRECTORY, BridgeWire.DIRECTORY_BYTES)); }
            catch (IllegalArgumentException corrupted) { return null; }
            if (record.index != index || record.generation != header.generation) return null;
            devices.add(new BridgeDevice(record));
        }
        return devices;
    }
    /**
     * A CAN encoder found by its own nickname (up to 12 characters of a-z, 0-9 and '-'), as set
     * in Hardware Manager. The name is looked up in the BRIDGE's directory when the selection is
     * made and then pinned to that encoder's UID, so a later rename does not move it. Until an
     * encoder with the name is heard the handle is invalid and a warning says so; two encoders
     * with the name leave it invalid too, with a warning naming both UIDs. Neither throws.
     */
    public CannibalEncoder encoderByName(String name, CannibalEncoder.Field... fields) {
        if (!BridgeWire.validNickname(name)) throw new IllegalArgumentException("encoder names are 1 to 12 characters of a-z, 0-9 and '-'");
        int mask = CannibalEncoder.Field.mask(fields);
        return new CannibalEncoder(this, row(1, BridgeWire.EITHER_BUS, 7, 8, 0, mask, name), mask);
    }
    /** A CAN encoder by its UID, as XXXX-XXXX-XXXX or 12 hex digits, on either CAN bus. */
    public CannibalEncoder encoderByUid(String uid, CannibalEncoder.Field... fields) {
        int mask = CannibalEncoder.Field.mask(fields);
        return new CannibalEncoder(this, row(1, BridgeWire.EITHER_BUS, 7, 8, uid(uid), mask, null), mask);
    }
    /** A quadrature encoder on one of the BRIDGE's local ports: 0 to 3 for ENC1 to ENC4. */
    public CannibalEncoder localEncoder(int port, CannibalEncoder.Field... fields) {
        int mask = CannibalEncoder.Field.mask(fields);
        if ((mask & BridgeWire.ABSOLUTE) != 0) throw new IllegalArgumentException("a local quadrature encoder has no absolute angle");
        return new CannibalEncoder(this, row(0, 0, 0, 0, port, mask, null), mask);
    }
    /** Pose and full-precision velocity. */
    public CannibalLocaliser getLocaliser() { return getLocaliser(CannibalLocaliser.Velocity.FULL); }
    public CannibalLocaliser getLocaliser(CannibalLocaliser.Velocity velocity) {
        return new CannibalLocaliser(this, row(3,0,0,0,0,velocity.fields,null), velocity.fields);
    }
    private synchronized BridgeWire.Row row(int kind, int bus, int type, int manufacturer, long identity, int mask, String name) {
        for (BridgeWire.Row row : rows) if (row.sameSource(kind,bus,type,manufacturer,identity,name)) {
            if ((row.fields & mask) != mask) { loseSelection(); row.fields |= mask; healthy = false; }
            return row;
        }
        BridgeWire.Row row = new BridgeWire.Row(-1,kind,bus,type,manufacturer,identity,mask,name);
        rows.add(row); loseSelection(); healthy = false; return row;
    }
    private static long uid(String display) {
        if (display == null || !display.matches("(?:[0-9A-Fa-f]{12}|[0-9A-Fa-f]{4}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{4})")) throw new IllegalArgumentException("UID must be 12 hex digits or XXXX-XXXX-XXXX");
        String hex = display.replace("-", ""); long result = 0;
        for (int i = 0; i < 6; i++) result |= (long)Integer.parseInt(hex.substring(i*2,i*2+2),16) << (i*8);
        return result;
    }
    private void loseSelection() {
        selected = false;
        // Any other operation on the port discards the hub's pending result, so a lost
        // selection simply forgets the prefetch rather than collecting it.
        prefetchPending = false;
        if (!commands.isEmpty() && commands.peek().sent) { commands.remove().state = Command.State.UNKNOWN; }
    }
    private void select() {
        if (info == null) info = new BridgeWire.Info(read(BridgeWire.INFO, 26));
        resolveNames();
        List<BridgeWire.Row> chosen = new ArrayList<>();
        for (BridgeWire.Row row : rows) { row.index = -1; if (row.resolved) { row.index = chosen.size(); chosen.add(row); } }
        selectedRows = chosen;
        if (chosen.isEmpty()) throw new IllegalStateException(rows.get(0).problem != null ? rows.get(0).problem : "no requested views");
        if (chosen.size() > info.maxViews) throw new IllegalStateException("selection exceeds bridge capacity");
        int id = BridgeWire.selectionId(chosen);
        int before = lastEvent;
        nextToken = 0; lastSequence = -1; lastHealthCounter = -1;
        write(BridgeWire.COMMAND, BridgeWire.begin(chosen.size()));
        for (BridgeWire.Row row : chosen) write(BridgeWire.COMMAND, BridgeWire.view(row));
        write(BridgeWire.COMMAND, BridgeWire.commit(id));
        for (BridgeWire.Row row : chosen) {
            write(BridgeWire.METADATA, new byte[]{(byte)row.index});
            try { row.metadata(read(BridgeWire.METADATA, 18), id); }
            catch (IllegalArgumentException failure) {
                // A rejected selection leaves the old one, or none: health says why.
                String reason = rejectionReason(readHealth(), before);
                throw reason == null ? failure : new IllegalStateException(reason);
            }
        }
        selectionId = id; selected = true;
    }
    /** True when sequence (1..255, wrapping to 1) is 1 to 127 steps after last. */
    static boolean isNewer(int sequence, int last) {
        int ahead = Math.floorMod(sequence - last, 255);
        return sequence != 0 && ahead >= 1 && ahead <= 127;
    }
    /**
     * Optional. Starts the next update()'s snapshot read and returns once the hub has
     * accepted it, about a millisecond, so the hub clocks the snapshot while the loop
     * does other work such as motor and servo writes. The next update() collects it.
     *
     * <p>Returns true when a read is pending. Returns false and does nothing when the
     * selection or a zero/pose/calibration command still has to be written first, or the
     * transport cannot split reads; update() then reads normally, so calling this can only
     * make a loop faster. Nothing else may use the BRIDGE's I²C port between prefetch()
     * and update().
     */
    public synchronized boolean prefetch() {
        if (prefetchPending) return true;
        if (rows.isEmpty() || !selected || info == null || !commands.isEmpty()) return false;
        int length = Math.min(chunkSize, BridgeWire.snapshotLength(selectedRows));
        long started = clock.getAsLong();
        try {
            if (!access.startRead(BridgeWire.SNAPSHOT, length)) return false;
        } catch (RuntimeException refused) {
            return false;
        }
        transactions++;
        prefetchPending = true; prefetchLength = length; prefetchStarted = started;
        return true;
    }
    private byte[] read(int register, int length) { transactions++; return access.read(register, length); }
    private void write(int register, byte[] payload) { transactions++; access.write(register, payload); }
    /**
     * One coherent refresh. A failed read (I2C error, corrupted or repeated snapshot, a reset
     * BRIDGE) changes nothing: handles keep the last accepted reading, valid until it is older
     * than the cache age limit, and a warning is logged at most once a second.
     */
    public synchronized boolean update() {
        long begun = clock.getAsLong(), setup = begun, command = begun, latched = begun, continued = begun, healthStarted = 0, healthEnded = 0;
        int bytesRead = 0; boolean succeeded = false, prefetched = false;
        transactions = 0;
        if (rows.isEmpty()) { lastError = "no requested views"; healthy = false; return false; }
        try {
            // A pending prefetch implies an installed selection (losing it forgets the
            // prefetch), so select() never touches the port ahead of the collect below.
            if (!selected) select();
            int length = BridgeWire.snapshotLength(selectedRows);
            if (length > BridgeWire.MAX_SNAPSHOT) throw new IllegalStateException("selection exceeds snapshot capacity");
            int firstLength = Math.min(chunkSize, length);
            if (length > info.maxSnapshot) throw new IllegalStateException("selection exceeds reported snapshot capacity");
            setup = command = latched = continued = clock.getAsLong();
            byte[] first = null;
            long started = setup;
            if (prefetchPending) {
                // Collect before anything else touches the port. A lost result returns null
                // and an old one is dropped; either way the normal read below runs instead.
                prefetchPending = false;
                boolean fresh = setup - prefetchStarted <= maximumPrefetchAgeNanos;
                transactions++;
                byte[] collected = access.finishRead(BridgeWire.SNAPSHOT, prefetchLength);
                if (collected != null && fresh && prefetchLength == firstLength) {
                    first = collected; started = prefetchStarted; prefetched = true;
                }
            }
            while (commands.peek() != null && commands.peek().row != null && commands.peek().row.index < 0) commands.remove().state = Command.State.UNKNOWN;
            Command pending = commands.peek();
            if (first == null) {
                if (pending != null) {
                    if (!pending.sent) { pending.token = ++nextToken; if (nextToken > 0xffffffffL) { loseSelection(); return false; } pending.sent = true; }
                    write(BridgeWire.COMMAND, pending.encode(selectionId));
                    command = latched = continued = clock.getAsLong();
                }
                started = command;
                first = read(BridgeWire.SNAPSHOT, firstLength);
            }
            latched = continued = clock.getAsLong(); bytesRead = firstLength;
            BridgeWire.require(first != null && first.length == firstLength, "short I2C read");
            byte[] bytes = new byte[length];
            System.arraycopy(first, 0, bytes, 0, firstLength);
            for (int offset = firstLength; offset < length;) {
                int count = Math.min(chunkSize, length - offset);
                write(BridgeWire.CONTINUATION, BridgeWire.offset(offset));
                byte[] part = read(BridgeWire.CONTINUATION, count);
                BridgeWire.require(part != null && part.length == count, "short I2C read");
                System.arraycopy(part, 0, bytes, offset, count); offset += count; bytesRead += count;
            }
            continued = clock.getAsLong();
            BridgeWire.Snapshot observed;
            try {
                observed = new BridgeWire.Snapshot(bytes, selectedRows, selectionId, started);
            } catch (BridgeWire.SnapshotRejected rejected) {
                // Health tells a BRIDGE reset or a lost selection from noise on the bus.
                healthStarted = clock.getAsLong();
                BridgeWire.Health report = readHealth();
                healthEnded = clock.getAsLong();
                if (report != null && (!report.selectionActive || report.selectionId != selectionId)) {
                    loseSelection();
                    throw new IllegalStateException("the BRIDGE has no selection (reset or configuration change); reselecting");
                }
                throw rejected;
            }
            // Prefetches that were dropped or never collected leave gaps, so accept any newer
            // sequence; only a snapshot served twice, or out of order, is refused.
            if (lastSequence >= 0 && !isNewer(observed.sequence, lastSequence))
                throw new IllegalStateException("repeated snapshot sequence " + observed.sequence);
            lastSequence = observed.sequence;
            snapshot = observed; healthy = true; lastError = ""; succeeded = true;
            // Health has news. Read it at most every 100 ms, so a source that keeps dropping
            // out costs one read per 100 ms rather than one per loop. A new selection's first
            // read, and a sent command's acknowledgement, are read at once.
            if (observed.health != lastHealthCounter) {
                Command waiting = commands.peek();
                long now = clock.getAsLong();
                if (lastHealthCounter < 0 || (waiting != null && waiting.sent) || now - lastHealthRead >= HEALTH_INTERVAL_NANOS) {
                    healthStarted = now;
                    if (readHealth() != null) lastHealthCounter = observed.health;
                    healthEnded = clock.getAsLong();
                }
            }
            // A missing name is looked for again when the directory changes, and now and then.
            if (waitingForNames() && (reportedGeneration != resolvedGeneration || clock.getAsLong() - lastDirectoryRead >= NAME_RETRY_NANOS))
                resolveNames();
            return true;
        } catch (RuntimeException failure) {
            // A glitch is a warning, not lost data: the last accepted snapshot stays in use
            // until hasFreshRead() ages it out.
            lastError = failure.getMessage(); failedUpdates++;
            long now = clock.getAsLong();
            if (now - lastReadWarningNanos >= 1_000_000_000L) {
                warnings.accept("CANnibal: BRIDGE read failed (" + lastError + "); using the previous reading");
                lastReadWarningNanos = now;
            }
            return false;
        }
        finally {
            long ended = clock.getAsLong();
            lastTiming = new UpdateTiming(succeeded, prefetched, Math.max(setup - begun, 0), Math.max(command - setup, 0), Math.max(latched - command, 0),
                Math.max(continued - latched, 0), Math.max(healthEnded - healthStarted, 0), ended - begun, transactions, bytesRead);
        }
    }
    /**
     * Reads register 0x23 and acts on its news: acknowledges applied commands and turns other
     * new events into warnings. The first read in a session only takes a baseline. Null when
     * the read failed its check.
     */
    private BridgeWire.Health readHealth() {
        lastHealthRead = clock.getAsLong();
        BridgeWire.Health report;
        try { report = new BridgeWire.Health(read(BridgeWire.HEALTH, BridgeWire.HEALTH_BYTES)); }
        catch (IllegalArgumentException unreadable) { return null; }
        boolean baseline = lastEvent < 0;
        // Events and uptime begin again with each BRIDGE run, so all of a new run's are news.
        // Health can go unread for a long time, so a run that has since passed the old uptime
        // is caught by uptime falling short of the time elapsed here, with slack for drift.
        long now = clock.getAsLong(), elapsedMs = (now - lastHealthNanos) / 1_000_000L;
        if (lastUptimeMs >= 0 && (report.uptimeMs < lastUptimeMs
            || report.uptimeMs + 1_000 + elapsedMs / 100 < lastUptimeMs + elapsedMs)) lastEvent = 0;
        lastUptimeMs = report.uptimeMs; lastHealthNanos = now;
        List<String> lines = new ArrayList<>();
        int news = 0;
        for (int i = report.events.length - 1; i >= 0; i--) {
            BridgeWire.Event event = report.events[i];
            if (lastEvent > 0 && !isNewer(event.sequence, lastEvent)) continue;
            news++;
            if (event.kind == BridgeWire.EVENT_COMMAND_APPLIED) acknowledge(event.token());
            else lines.add("CANnibal: " + describe(event));
        }
        if (!baseline) {
            int expected = lastEvent > 0 ? Math.floorMod(report.latestEvent - lastEvent, 255) : report.latestEvent;
            if (expected > news) warnings.accept("CANnibal: " + (expected - news) + " BRIDGE health events were missed");
            for (String line : lines) warnings.accept(line);
        }
        lastEvent = report.latestEvent;
        reportedGeneration = report.directoryGeneration;
        List<String> recent = new ArrayList<>();
        for (int i = report.events.length - 1; i >= 0; i--) recent.add(describe(report.events[i]));
        health = new BridgeHealth(report, recent);
        return report;
    }
    /** The newest selection rejection after event `after`, in words, or null when there is none. */
    private String rejectionReason(BridgeWire.Health report, int after) {
        if (report == null) return null;
        for (BridgeWire.Event event : report.events)
            if (event.kind == BridgeWire.EVENT_SELECTION_REJECTED && (after <= 0 || isNewer(event.sequence, after))) return describe(event);
        return null;
    }
    private void acknowledge(long token) {
        Command pending = commands.peek();
        if (pending != null && pending.sent && pending.token == token) { pending.state = Command.State.APPLIED; commands.remove(); }
    }
    private String label(int view) {
        return view < selectedRows.size() ? selectedRows.get(view).label() : "view " + view;
    }
    private String describe(BridgeWire.Event event) {
        String subject = event.view != BridgeWire.NO_VIEW ? label(event.view) : "Device " + BridgeWire.uidText(event.uid());
        boolean localiser = event.view < selectedRows.size() && selectedRows.get(event.view).kind == 3;
        if (localiser && event.kind == BridgeWire.EVENT_SOURCE_RESTARTED) return "Localiser pose continuity was interrupted; it continues from where it was";
        if (localiser && event.kind == BridgeWire.EVENT_SOURCE_MISSING) return event.detail == 1
            ? "Localiser has had no usable pose since selection; check its mode and tracking sources in Hardware Manager"
            : "Localiser has no usable pose: a tracking source or the IMU is missing or stale";
        if (localiser && event.kind == BridgeWire.EVENT_SOURCE_RETURNED) return "Localiser pose is usable again";
        switch (event.kind) {
            case BridgeWire.EVENT_BRIDGE_RESET: return "BRIDGE restarted (reset cause: " + BridgeHealth.resetCause(event.detail) + ")";
            case BridgeWire.EVENT_SOURCE_RESTARTED: return subject + " restarted" + ((event.detail & 1) != 0 ? "; whole revolutions may be missing from its count" : "");
            case BridgeWire.EVENT_SOURCE_MISSING: return subject + (event.detail == 1 ? " has not been seen since selection" : " is missing");
            case BridgeWire.EVENT_SOURCE_RETURNED: return subject + " is back";
            case BridgeWire.EVENT_UID_UNRESOLVED: return subject + " was not found on CAN";
            case BridgeWire.EVENT_UID_DUPLICATE: return subject + " is on both CAN buses";
            case BridgeWire.EVENT_NUMBER_CONFLICT: return "device number " + (event.detail & 63) + " on CAN" + ((event.detail >> 6) + 1)
                + " is used by more than one device, including " + BridgeWire.uidText(event.uid());
            case BridgeWire.EVENT_CAN_BUS_OFF: return "CAN" + (event.detail + 1) + " bus-off";
            case BridgeWire.EVENT_CAN_RECOVERED: return "CAN" + (event.detail + 1) + " recovered from bus-off";
            case BridgeWire.EVENT_SELECTION_REJECTED: return "BRIDGE rejected the selection: " + rejection(event);
            case BridgeWire.EVENT_SELECTION_INVALIDATED: return "BRIDGE " + (event.detail == 2 ? "local ports" : "configuration") + " changed; reselecting";
            case BridgeWire.EVENT_IMU_CALIBRATING: return "IMU calibrating; keep the robot still";
            case BridgeWire.EVENT_IMU_CALIBRATED: return "IMU calibrated";
            case BridgeWire.EVENT_IMU_CONFIG_FAULT: return BridgeHealth.configurationFault(event);
            case BridgeWire.EVENT_COMMAND_APPLIED: return "command " + event.token() + " applied";
            default: return "BRIDGE event " + event.kind;
        }
    }
    private String rejection(BridgeWire.Event event) {
        String view = event.view != BridgeWire.NO_VIEW ? label(event.view) : "a view";
        switch (event.detail) {
            case BridgeWire.REJECT_UNSUPPORTED: return view + " is not supported by this BRIDGE firmware; update the BRIDGE firmware";
            case BridgeWire.REJECT_LOCAL_PORT: return view + " uses a local port this BRIDGE does not have";
            case BridgeWire.REJECT_TOO_LARGE: return "the selection exceeds the BRIDGE's snapshot size";
            case BridgeWire.REJECT_MISMATCH: return "the selection arrived corrupted";
            default: return "malformed selection";
        }
    }
    /** Where the most recent update() spent its time, or null before the first one. */
    public UpdateTiming getLastUpdateTiming() { return lastTiming; }
    /**
     * Phases of one update(), in nanoseconds of the client's clock. A phase that did
     * not run, or was cut short by a failure, reads zero. Each I²C transaction is one
     * SDK read (register write, repeated start and read) or one SDK write.
     */
    public static final class UpdateTiming {
        public final boolean succeeded;
        /** The snapshot came from a prefetch() read collected by this update. */
        public final boolean prefetched;
        /** Identity, selection and units checks; zero once the selection is installed. */
        public final long setupNanos;
        /** Writing a queued zero, pose or calibration command. */
        public final long commandNanos;
        /** Reading register 0x20, which latches the snapshot, or collecting it after prefetch(): the whole read for selections of 100 bytes or less. */
        public final long snapshotReadNanos;
        /** Offset writes and continuation reads for snapshots over the chunk size. */
        public final long continuationNanos;
        /**
         * Reading the health register: when the snapshot's health counter moved (at most every
         * 100 ms, but at once while a command waits for its acknowledgement), or a snapshot was refused.
         */
        public final long healthNanos;
        public final long totalNanos;
        public final int transactions, snapshotBytes;
        UpdateTiming(boolean succeeded, boolean prefetched, long setupNanos, long commandNanos, long snapshotReadNanos, long continuationNanos,
                     long healthNanos, long totalNanos, int transactions, int snapshotBytes) {
            this.succeeded = succeeded; this.prefetched = prefetched; this.setupNanos = setupNanos; this.commandNanos = commandNanos;
            this.snapshotReadNanos = snapshotReadNanos; this.continuationNanos = continuationNanos; this.healthNanos = healthNanos;
            this.totalNanos = totalNanos; this.transactions = transactions; this.snapshotBytes = snapshotBytes;
        }
    }
    public boolean hasFreshRead() { BridgeWire.Snapshot s = snapshot; return healthy && s != null && clock.getAsLong() - s.hostNanos <= maximumCacheAgeNanos; }
    public String getLastError() { return lastError; }
    /** Failed update() calls since construction; each kept the previous reading. */
    public synchronized long getFailedUpdateCount() { return failedUpdates; }
    /** The last accepted snapshot's sequence, 1 to 255, or 0 before one. */
    public int getSnapshotSequence() { BridgeWire.Snapshot s = snapshot; return s == null ? 0 : s.sequence; }
    public double getReadAgeSeconds() { BridgeWire.Snapshot s = snapshot; return s == null ? Double.POSITIVE_INFINITY : (clock.getAsLong() - s.hostNanos) / 1e9; }
    BridgeWire.Sample sample(BridgeWire.Row row) {
        BridgeWire.Snapshot s = snapshot;
        int index = s == null ? -1 : s.rows.indexOf(row);
        return index < 0 ? null : s.samples[index];
    }
    synchronized Command enqueue(int opcode, int... values) { return enqueue(new Command(opcode, values, null)); }
    synchronized Command enqueueZero(BridgeWire.Row row) { return enqueue(new Command(6, new int[1], row)); }
    private Command enqueue(Command command) {
        if (commands.size() >= 32) return refuse(command, "32 commands are already waiting for the BRIDGE to apply them");
        lastRefusal = null; commands.add(command); return command;
    }
    /** A command that was never sent, logged as a warning once a second while a loop repeats it. */
    synchronized Command refuse(String why) { return refuse(new Command(0, new int[0], null), why); }
    private Command refuse(Command command, String why) {
        command.state = Command.State.REFUSED;
        long now = clock.getAsLong();
        if (!why.equals(lastRefusal) || now - lastRefusalNanos >= 1_000_000_000L) {
            warnings.accept("CANnibal: command refused: " + why); lastRefusal = why; lastRefusalNanos = now;
        }
        return command;
    }
    public Command calibrateImu() { return enqueue(5); }
    /**
     * A queued zero, pose or calibration command. It is sent with update() until the BRIDGE
     * reports it applied in its health events; the snapshot whose health counter first shows
     * that already reflects it. UNKNOWN means the selection was lost first. REFUSED means it
     * was never sent: its values were not finite or in range, or 32 commands were already
     * waiting because the BRIDGE is not applying them. A refusal is a RobotLog warning, never
     * an exception.
     */
    public static final class Command {
        public enum State { PENDING, APPLIED, UNKNOWN, REFUSED }
        private final int opcode; private final int[] values; private final BridgeWire.Row row; private long token; private boolean sent;
        private volatile State state = State.PENDING;
        private Command(int opcode, int[] values, BridgeWire.Row row) { this.opcode = opcode; this.values = values; this.row = row; }
        public State getState() { return state; }
        public boolean isApplied() { return state == State.APPLIED; }
        /** A zero names its view by the index the encoder has in the selection it is sent with. */
        byte[] encode(int selectionId) {
            if (row != null) values[0] = row.index;
            return BridgeWire.runtime(opcode,selectionId,token,values);
        }
    }
}
