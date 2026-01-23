package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TestLs")
public class TestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        LimitSwitchHome test = new LimitSwitchHome(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Status", "Homing Complete");
            telemetry.update();
        }
    }
}