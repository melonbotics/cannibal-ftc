# CANnibal FTC library

Read CANnibal Nano Encoders, the BRIDGE's quadrature encoder ports and its
onboard localiser from an FTC robot, through one I²C port on a REV Control Hub.

Use one `CannibalBridge` for each BRIDGE. Encoder and localiser handles share
its selected snapshot. `update()` refreshes that snapshot; getters read the
cache and generate no I²C traffic. Odometry runs in the BRIDGE firmware.

The library is Java 8, built against FTC SDK 12.0.0. It needs BRIDGE firmware
with I²C interface revision 2; if the firmware is older or newer,
`bridge.client().getLastError()` says which side to update.

## Install

Download `cannibal-ftc-0.1.0.jar` from this repository's Releases and copy it
into `TeamCode/libs`. If your project does not already include every JAR in
that directory, add this to the `dependencies` block of `TeamCode/build.gradle`:

```gradle
implementation files('libs/cannibal-ftc-0.1.0.jar')
```

Alternatively, copy `src/main/java` into `TeamCode/src/main/java`. Do not do
both: they define the same classes. The JAR contains only CANnibal code; the
FTC SDK comes from your Robot Controller project.

In the robot configuration, select **CANnibal Bridge** on the Control Hub I²C
port the BRIDGE is wired to and name it `bridge`. Its default seven-bit address
is `0x42`.

Before running an OpMode, use Hardware Manager to set up the BRIDGE and its
Nano Encoders: CPR, filtering, odometry geometry, and the nicknames your code
finds encoders by. The library reads these settings and never rewrites them.
Put Nano Encoders you want to find by name on CAN1 or CAN2 (see below).

Start from [the example OpMode](examples/CannibalBridgeExample.java).

## Robot code

```java
import com.melonbotics.cannibal.CannibalEncoder.Field;

CannibalBridge bridge = hardwareMap.get(CannibalBridge.class, "bridge");
CannibalEncoder azimuth = bridge.encoderByName("fl-azimuth", Field.ABSOLUTE, Field.VELOCITY);
CannibalEncoder drive = bridge.encoderByUid("3a2c-8aec-75f0");
CannibalEncoder odometry = bridge.localEncoder(0, Field.POSITION_EXTENDED, Field.VELOCITY);
CannibalLocaliser localiser = bridge.getLocaliser();

// Once per control iteration:
if (bridge.update() && localiser.isValid()) {
    CannibalLocaliser.Pose pose = localiser.getPose();
    // pose.x / pose.y are metres; pose.heading is radians, CCW positive.
}
```

Create handles before the loop. There is one encoder handle type and three
ways to make one:

- `encoderByName("fl-azimuth")` finds a Nano Encoder by the nickname set in
  Hardware Manager: up to 12 characters of a-z, 0-9 and '-'. The library looks
  the name up in the BRIDGE's device directory when it makes its selection and
  then pins the handle to that encoder's UID, so renaming it later does not
  move the handle. Until an encoder with the name is heard, the handle is
  invalid, `getProblem()` says why, and RobotLog has a warning; the library
  looks again whenever the directory changes. Two encoders with the same name
  leave the handle invalid too, without throwing: the warning and
  `getProblem()` name both UIDs, and the handle stays invalid until all but one
  are renamed.
- `encoderByUid("3a2c-8aec-75f0")` selects a Nano Encoder by the UID Hardware
  Manager shows; twelve hex digits without dashes work too.
- `localEncoder(port)` reads a quadrature encoder on a local port. The BRIDGE
  has four, ENC1–ENC4, numbered 0–3.

CAN encoders are found on any of the BRIDGE's CAN buses and followed across
device-number and bus changes. `bridge.readDirectory()` lists every CANnibal
device the BRIDGE has heard on CAN1 or CAN2, with its name, UID, bus and
number; it costs one I²C read per device, so use it at setup or for
telemetry. A device on the BRIDGE's CAN3 works through `encoderByUid` and as a
tracking source, but this interface revision cannot name its bus, so it is
not in the directory and `encoderByName` cannot find it. The BRIDGE always lists
itself first, so an empty list means the read failed (`getLastError()` says
why).

Each handle declares the fields it reads, and only those bytes are read.
Every byte costs about 72 µs at 400 kHz, so declare what the robot uses:

| Field | Bytes | Getters |
| --- | --- | --- |
| `POSITION` | 4 | `getPosition()` (counts modulo 2³²), `getPositionRadians()` |
| `VELOCITY` | 2 | `getVelocity()` (counts/s), `getVelocityRadiansPerSecond()`; 1/128 rev/s up to ±15,360 RPM |
| `POSITION_EXTENDED` | 8 | `getPosition()` as 64-bit counts |
| `VELOCITY_EXTENDED` | 4 | velocity getters in whole counts/s |
| `ABSOLUTE` | 2 | `getAbsoluteCounts()`, `getAbsoluteAngleRadians()`; CAN encoders only |

With no fields named, a handle reads `POSITION` and `VELOCITY`. A swerve
azimuth needs only `ABSOLUTE` and `VELOCITY`: four bytes. Calling a getter
for a field the handle did not declare throws an `IllegalStateException`
saying which field to add. Handles for the same source share one selection
entry with the union of their fields. There are at most 32 selected sources.
Counts are *decoded* counts per revolution, including quadrature edges; Nano
Encoder units are 16,384 counts/revolution. The library consumes the current
angle profile, including advisory magnetic-field flags and unavailable
velocity range-end codes.

| Localiser | Selected data | Main getters |
| --- | --- | --- |
| `getLocaliser()` | pose, field-frame velocity in µm/s and µrad/s (24 bytes) | `getPose()`, `getVelocity()`, `setPose(x,y,heading)` |
| `getLocaliser(Velocity.COMPACT)` | pose, velocity in mm/s and mrad/s (18 bytes) | as above |
| `getLocaliser(Velocity.NONE)` | pose only (12 bytes) | `getPose()`, `setPose(x,y,heading)` |

Pose coordinates are +x forward, +y left, positive heading counter-clockwise.
Two-wheel mode requires calibrated, populated IMU channels and configured wheel
geometry; three-wheel mode obtains heading from the tracking wheels. Sources
may be local or on any CAN bus. Select each tracking role in Hardware
Manager as a local port or an explicit Nano Encoder UID before expecting
`localiser.isValid()`. Firmware resolves CAN tracking UIDs across every port;
device numbers and nicknames are presentation metadata for these assignments.

## Refresh and runtime controls

A snapshot fitting 100 bytes uses one read after setup. Larger selections use
frozen continuation reads. Setup checks the BRIDGE's identity and interface
revision, installs the selection and reads each view's units. Every snapshot's
check is seeded with the selection's ID, so a read that fails the check is
either corrupted or from a BRIDGE that reset or lost the selection; the library
reads the BRIDGE's health register to tell which, and selects again when
needed. A read that fails, from an I²C error or a corrupted snapshot, changes
nothing: handles keep the last accepted reading, valid until it is older than
the cache age limit, and a RobotLog warning is logged at most once a second.
`bridge.client().getFailedUpdateCount()` counts them. The library never accepts
the SDK's all-zero error fill.

`isValid()` means there is a usable number: the source is present, fresh and
in range, and the last bridge refresh is recent. The host cache expires after
100 ms by default; `bridge.client().setMaximumCacheAgeMillis(...)` adjusts that
separate host polling limit. `bridge.client().getLastError()` explains
transport/setup failure; `update()` can succeed while an individual missing or
calibrating source is invalid. Treat pose/count values from an invalid handle
as diagnostic history.

The BRIDGE reports what happens to its sources as health events: a restarted
or missing encoder, CAN bus-off, a rejected selection, IMU calibration. Each
snapshot carries a health counter; when it moves, `update()` reads the health
register, at most once every 100 ms so a source that keeps dropping out cannot
add a read to every loop, but at once while a command waits for its
acknowledgement. It logs each new event as a RobotLog warning such as
`CANnibal: Encoder 3a2c-8aec-75f0 restarted`. `bridge.getHealth()` returns
the state from that read: uptime, reset cause, supply voltages, CAN state and
error counts, and the recent events. A health read costs about as much as a
snapshot read and `getLastUpdateTiming().healthNanos` shows it.

`encoder.zero()`, `localiser.setPose(...)` and `bridge.calibrateImu()` queue
runtime commands for `update()`. Each returns a `BridgeClient.Command` with
`PENDING`, `APPLIED`, `UNKNOWN` or `REFUSED` state. Commands are sent one at a time until
the BRIDGE reports them applied in a health event; retries keep the same
token, and the snapshot that first shows the event already reflects the
command. If selection is lost after a command was sent and before it was
reported, the result is `UNKNOWN`: the library does not replay an
unacknowledged zero into a new reset/selection epoch. The robot decides
whether another command is appropriate. Queueing a command is not completion.
A pose that is not finite, or a 33rd command while 32 wait because the BRIDGE
is not applying them, comes back `REFUSED` with a RobotLog warning.

Nothing a loop calls throws because of the BRIDGE, its devices or a computed
value, so a match never stops on a library exception: failures show as
`update()` returning false, invalid handles, `getLastError()` and RobotLog
warnings. Only mistakes in the code itself throw, such as an invalid UID string
or reading a field a handle was not declared with.

`encoder.zero()` makes `getPosition()` read zero from that moment. It does not
change the absolute angle: `getAbsoluteCounts()` and `getAbsoluteAngleRadians()`
always report the magnet's angle within the turn, so keep a swerve module's
offset in your own code. Zeroing writes nothing to the encoder and leaves
odometry intact. A pose reset creates new odometry baselines.
A restarted source stays valid: a rebooted Nano Encoder, lost CAN frames,
bus-off or a rail dip never needs `zero()`. The bridge carries each count on
from where it was (a Nano by its absolute angle) and keeps the pose,
integrating across the gap. Calibration requires stationary sensors; two-wheel
pose is invalid while it runs and then continues from where it was. Fresh boot
starts volatile pose anew and does not restore a field pose from flash.

The bridge runs its hub port at 400 kHz, set again whenever the device is
initialised for an OpMode. If another device on the same port only supports
100 kHz, call `bridge.setI2cSpeed(CannibalBridge.I2cSpeed.STANDARD_100K)`.
`bridge.client().getLastUpdateTiming()` reports how long the last `update()`
spent in setup, a queued command, the snapshot read and any continuation reads,
plus its I²C transaction count, and whether it collected a prefetch. Use it to
see what a loop is waiting for.

## Faster loops: prefetch

Most of an `update()` is the hub clocking the snapshot over I²C on its own, about
4 ms of the 7 ms a 60-byte snapshot takes at 400 kHz. `bridge.prefetch()` starts
the next snapshot and returns in about a millisecond, so the hub does that
transfer while your loop sends its motor and servo commands. The next `update()`
collects it with one status query:

```java
while (opModeIsActive()) {
    bridge.update();     // collect: the prefetched snapshot, or a fresh read
    // read handles and compute outputs
    bridge.prefetch();   // start the next snapshot
    // set motor powers and servo positions; they run during the transfer
}
```

On a Control Hub, four motor writes per loop took the loop from 11.7 ms to
8.6 ms. `prefetch()` uses only the hub's normal acknowledged commands, one at a
time, as the SDK's own I²C reads do, so it needs nothing special for an
Expansion Hub on RS485 (not yet tested there).

- `update()` on its own behaves exactly as before, so `prefetch()` can only make
  a loop faster. It returns false and does nothing when setup or a queued
  command has to go first, or the hub cannot split reads; the next `update()`
  then reads normally. If the hub lost the prefetched read, `update()` reads
  again. A prefetch older than 50 ms is dropped and read fresh
  (`bridge.client().setMaximumPrefetchAgeMillis(...)`).
- The numbers are from when `prefetch()` ran. Put only the loop's writes
  between `prefetch()` and `update()`.
- Nothing else may use the BRIDGE's I²C port. Devices on other ports are free
  while the transfer runs.
- A command such as `zero()` queued after `prefetch()` goes out with the
  following `update()`, so its acknowledgement arrives one loop later.

Use `bridge.setI2cAddress(...)` if Hardware Manager changed the BRIDGE's
address. `bridge.client().setReadChunkSize(...)` lowers the normal 100-byte
read chunk (32–100 bytes).

## Without a Control Hub

`BridgeClient` and `I2cAccess` have no Android dependencies, so the client runs
over any transport that implements `I2cAccess`. A monotonic nanosecond clock
can be injected into `BridgeClient` for deterministic tests.

## Licence

MIT; see [LICENSE](LICENSE).
