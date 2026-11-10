package org.firstinspires.ftc.teamcode;

import com.melonbotics.cannibal.CannibalEncoder;
import com.melonbotics.cannibal.CannibalEncoder.Field;
import com.melonbotics.cannibal.CannibalLocaliser;
import com.melonbotics.cannibal.ftc.CannibalBridge;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="CANnibal Bridge example")
public final class CannibalBridgeExample extends LinearOpMode {
    @Override public void runOpMode() throws InterruptedException {
        CannibalBridge bridge = hardwareMap.get(CannibalBridge.class, "bridge");
        // A Nano Encoder named "fl-azimuth" in Hardware Manager, on either CAN bus: its angle
        // within the turn and its speed are all a swerve azimuth needs, four bytes a loop.
        CannibalEncoder azimuth = bridge.encoderByName("fl-azimuth", Field.ABSOLUTE, Field.VELOCITY);
        // A quadrature encoder on local port 0, read with 64-bit counts.
        CannibalEncoder drive = bridge.localEncoder(0, Field.POSITION_EXTENDED, Field.VELOCITY);
        CannibalLocaliser localiser = bridge.getLocaliser(CannibalLocaliser.Velocity.COMPACT);
        waitForStart();
        while (opModeIsActive()) {
            boolean refreshed = bridge.update();
            telemetry.addData("bridge", refreshed ? "read" : bridge.client().getLastError());
            telemetry.addData("azimuth", azimuth.isValid() ? String.format("%.3f rad", azimuth.getAbsoluteAngleRadians())
                : azimuth.getProblem() != null ? azimuth.getProblem() : "invalid");
            telemetry.addData("drive counts", drive.getPosition());
            telemetry.addData("pose valid", localiser.isValid());
            if (localiser.isValid()) {
                CannibalLocaliser.Pose pose = localiser.getPose();
                telemetry.addData("pose m, rad", "%.3f %.3f %.3f", pose.x, pose.y, pose.heading);
            }
            telemetry.update();
        }
    }
}
