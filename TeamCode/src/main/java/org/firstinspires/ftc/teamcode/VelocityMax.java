package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Autonomous
public class VelocityMax extends LinearOpMode {
    DcMotorEx wheel;
    @Override
    public void runOpMode() {
        wheel = hardwareMap.get(DcMotorEx.class, "wheel");
        wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while(opModeIsActive()) {
            double velocity = wheel.getVelocity(DEGREES);
            wheel.setPower(1);
            telemetry.addData("V: ", velocity);
            Log.d("velocity", "V: " + velocity);
            telemetry.update();
        }

        if (isStopRequested()) {
            wheel.setPower(0);
        }
    }
}