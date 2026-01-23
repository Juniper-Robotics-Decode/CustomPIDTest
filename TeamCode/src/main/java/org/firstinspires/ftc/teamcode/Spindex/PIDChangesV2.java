package org.firstinspires.ftc.teamcode.Spindex;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="PIDv2", group="Iterative")
public class PIDChangesV2 extends LinearOpMode {
    DcMotorEx wheel;
    final double CPR = 537.7;
    final double POCKET_ANGLE = 120.0;

    double kP = 0.015;
    double kD = 0.0003;
    double kS = 0.12;
    double tolerance = 1.0;


    double velP = 0.0025;
    double velI = 0.012;
    double velD = 0.0001;
    double velF = 1.0 / 245.0;
    double targetSpinSpeed = 80.0;


    double velIntegral = 0;
    double velLastError = 0;
    double lastVelTime = 0;
    boolean velocityMode = false;
    double targetAngle = 0;
    boolean lastDpadState = false;
    double lastIndexTime = 0;

    @Override
    public void runOpMode() {
        wheel = hardwareMap.get(DcMotorEx.class, "wheel");
        wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        lastVelTime = getRuntime();

        while (opModeIsActive()) {

            if (gamepad1.dpad_down && !lastDpadState) {
                velocityMode = !velocityMode;
                if (!velocityMode) {
                    double currentPos = (wheel.getCurrentPosition() / CPR) * 360.0;
                    targetAngle = Math.round(currentPos / POCKET_ANGLE) * POCKET_ANGLE;
                    lastIndexTime = getRuntime();
                }
                velIntegral = 0;
                velLastError = 0;
            }
            lastDpadState = gamepad1.dpad_down;

            if (velocityMode) {
                PIDVel(targetSpinSpeed);
            } else {
                double currentPosition = (wheel.getCurrentPosition() / CPR) * 360.0;
                double error = targetAngle - currentPosition;


                if (Math.abs(error) <= tolerance && getRuntime() - lastIndexTime > 0.4) {
                    targetAngle += POCKET_ANGLE;
                    lastIndexTime = getRuntime();
                }

                PIDPos(targetAngle);
            }

            telemetry.addData("Mode", velocityMode ? "Velocity" : "Position");
            telemetry.addData("Target", targetAngle);
            telemetry.addData("Actual", (wheel.getCurrentPosition() / CPR) * 360.0);
            telemetry.update();
        }
    }

    public void PIDVel(double targetVel) {
        double now = getRuntime();
        double dt = now - lastVelTime;
        if (dt <= 0) dt = 0.004;

        double vel = wheel.getVelocity(DEGREES);
        double error = targetVel - vel;

        velIntegral = Range.clip(velIntegral + error * dt, -100, 100);
        double derivative = (error - velLastError) / dt;

        double power = (velP * error) + (velI * velIntegral) + (velD * derivative) + (velF * targetVel);
        wheel.setPower(Range.clip(power, -1, 1));

        velLastError = error;
        lastVelTime = now;
    }

    public void PIDPos(double targetDeg) {
        double currentDeg = (wheel.getCurrentPosition() / CPR) * 360.0;
        double error = targetDeg - currentDeg;
        double currentVel = wheel.getVelocity(DEGREES);

        if (Math.abs(error) <= tolerance) {
            wheel.setPower(0);
            return;
        }

        double pOutput = kP * error;
        double dOutput = kD * currentVel;
        double output = pOutput - dOutput;

        output += Math.signum(error) * kS;

        wheel.setPower(Range.clip(output, -0.5, 0.5));
    }
}