package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class PIDChanges extends LinearOpMode {
    DcMotorEx wheel;
    double P = 0.012;
    double D = 0.00013;
    double F = 0.0;
    double velP = 0.002;
    double velI = 0.01;
    double velD = 0.0001;
    double velF = 1.0 / 245.0;

    double tolerance = 1.5;
    double CPR = 537.7; // gobilda 5203 series
    double velIntegral = 0;
    double velLastError = 0;
    double lastTime = 0;
    double lastIndexTime = 0;
    int pocket = 0;
    double targetAngle = 0;
    boolean velocityMode = false;
    boolean lastDpadState = false;

    @Override
    public void runOpMode() throws InterruptedException {
        wheel = hardwareMap.get(DcMotorEx.class, "wheel");
        wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        lastTime = getRuntime();
        lastIndexTime = getRuntime();
        while (opModeIsActive()) {
            boolean dpadPressed = gamepad1.dpad_down;
            if (dpadPressed && !lastDpadState) {
                velocityMode = !velocityMode;
                if (!velocityMode) {
                    targetAngle = (wheel.getCurrentPosition() / CPR) * 360.0;
                }
                velIntegral = 0;
                velLastError = 0;
            }
            lastDpadState = dpadPressed;
            if (velocityMode) {
                PIDVel(40);
            } else {
                double currentPosition = (wheel.getCurrentPosition() / CPR) * 360.0;
                double error = targetAngle - currentPosition;
                if (Math.abs(error) <= tolerance && getRuntime() - lastIndexTime > 0.5) {
                    pocket++;
                    if (pocket >= 3) pocket = 0;

                    targetAngle += 120;
                    lastIndexTime = getRuntime();
                }
                PIDPos(targetAngle);
            }
            telemetry.addData("Mode", velocityMode ? "Velocity" : "Position");
            telemetry.addData("Pocket", pocket);
            telemetry.addData("Target Angle", targetAngle);
            telemetry.update();
        }
        wheel.setPower(0);
    }

    public void PIDVel(double target) {
        double currentTime = getRuntime();
        double timeChange = currentTime - lastTime;
        if (timeChange <= 0) timeChange = 0.004;
        double currentVelocity = wheel.getVelocity(DEGREES);
        double errorP = target - currentVelocity;
        velIntegral = Range.clip(velIntegral + (errorP * timeChange), -100, 100); // Tighter clip on integral
        double errorD = (errorP - velLastError) / timeChange;
        double correction = (velP * errorP) + (velI * velIntegral) + (velD * errorD) + (velF * target);
        wheel.setPower(Range.clip(correction, -1.0, 1.0));
        velLastError = errorP;
        lastTime = currentTime;
        telemetry.addData("Vel Target", target);
        telemetry.addData("Vel Actual", currentVelocity);
    }
    public void PIDPos(double target) {
        double currentPosition = (wheel.getCurrentPosition() / CPR) * 360.0;
        double error = target - currentPosition;
        double velocity = wheel.getVelocity(DEGREES);
        double correction = (P * error) - (D * velocity);
        double minPower = 0.18;
        if (Math.abs(error) > tolerance) {
            if (correction > 0) correction += minPower;
            else if (correction < 0) correction -= minPower;
        }
        wheel.setPower(Range.clip(correction, -0.3, 0.3));
        telemetry.addData("Pos Error", error);
        telemetry.addData("Motor Power", correction);
    }
}