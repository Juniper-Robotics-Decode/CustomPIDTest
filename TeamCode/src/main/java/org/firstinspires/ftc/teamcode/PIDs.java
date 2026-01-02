package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import android.util.Log;

@TeleOp
public class PIDs extends LinearOpMode {
    DcMotorEx wheel;
    double P = 0.007; //increase if its taking too long to get to target velocity
    double I = 0.018; //change if its not getting close enough to the target velocity
    double D = 0.00013; //change if its jumpy
    double F = 1 / 245; //dont touch
//    double toleranceV = 5;
    double tolerance = 2;
    double currentVelocity = 0;
    double currentPosition = 0;
    double CPR = 145.1;//do not touch(motor specific number)(145.1 for gobilda 5203 series motor)

    double lastErrorP = 0;
    double lastErrorI = 0;
    double lastTime = getRuntime();

    @Override
    public void runOpMode() throws InterruptedException {
        wheel = hardwareMap.get(DcMotorEx.class, "wheel");
        wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            PIDPos(120);
            wait(300);
            PIDPos(120);
            wait(300);
            PIDPos(120);
            while(gamepad1.dpad_down){
                PIDVel(100);
            }
        }
        if (isStopRequested()) {
            wheel.setPower(0);
        }
    }
    public void PIDVel(double target){
            double currentTime = getRuntime();
            double timeChange = currentTime - lastTime;
            if (timeChange <= 0) {
                timeChange = 0.004;
            }
            currentVelocity = wheel.getVelocity(DEGREES);
            double errorP = calculateErrorP(target, currentVelocity);
            double errorD = calculateErrorD(errorP, lastErrorP, timeChange);
            double errorI = calculateErrorI(errorP, lastErrorI, timeChange);
            double correction = (P * errorP) + (I * errorI) + (D * errorD) + (F * target);
            wheel.setPower(Range.clip(correction, -1, 1));
            lastErrorP = errorP;
            lastErrorI = errorI;
            lastTime = currentTime;

            telemetry.addData("errorP: ", errorP);
            telemetry.addData("lastErrorP", lastErrorP);
            telemetry.addData("errorI: ", errorI);
            telemetry.addData("errorD: ", errorD);
            Log.d("correctionP: ", String.valueOf(P * errorP));
            Log.d("correctionI: ", String.valueOf(I * errorI));
            Log.d("correctionD: ", String.valueOf(D * errorD));
//            wheel.setPower(1);
            telemetry.addData("velocity: ", currentVelocity);
            telemetry.addData("correction: ", correction);
            telemetry.addData("time: ", timeChange);
            Log.d("velocityP", "errorP: " + errorP);
            Log.d("velocityI", "errorI: " + errorI);
            Log.d("velocityD", "errorD: " + errorD);
            Log.d("velocityPC", "correctionP: " + (P * errorP));
            Log.d("velocityIC", "correctionI: " + (I * errorI));
            Log.d("velocityDC", "correctionD: " + (D * errorD));
            Log.d("velocityV", "velocity: " + currentVelocity);
            Log.d("velocityC", "power: " + correction);
            Log.d("velocityT", "time: " + timeChange);
            Log.d("velocityPL", "lastErrorP: " + lastErrorP);
            Log.d("velocityE", "precision: " + (100 * currentVelocity / target));
            telemetry.update();
        }

    public void PIDPos(double target){
        wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(Math.abs(target - currentPosition) > tolerance){
            double currentTime = getRuntime();
            double timeChange = currentTime - lastTime;
            if (timeChange <= 0) {
                timeChange = 0.004;
            }
            double currentPosition = (wheel.getCurrentPosition() / CPR) * 360;//converting to degrees
            double errorP = calculateErrorP(target, currentPosition);
            double errorD = calculateErrorD(errorP, lastErrorP, timeChange);
            double errorI = calculateErrorI(errorP, lastErrorI, timeChange);
            double correction = (P * errorP) + (I * errorI) + (D * errorD);
            wheel.setPower(Range.clip(correction, -1, 1));
            lastErrorP = errorP;
            lastErrorI = errorI;
            lastTime = currentTime;

            telemetry.addData("PErrorP: ", errorP);
            telemetry.addData("PLastErrorP", lastErrorP);
            telemetry.addData("PErrorI: ", errorI);
            telemetry.addData("PErrorD: ", errorD);
            Log.d("PCorrectionP: ", String.valueOf(P * errorP));
            Log.d("PCorrectionI: ", String.valueOf(I * errorI));
            Log.d("PCorrectionD: ", String.valueOf(D * errorD));
//            wheel.setPower(1);
            telemetry.addData("Position: ", currentPosition);
            telemetry.addData("PCorrection: ", correction);
            telemetry.addData("PTime: ", timeChange);
            Log.d("PositionP", "errorP: " + errorP);
            Log.d("PositionI", "errorI: " + errorI);
            Log.d("PositionD", "errorD: " + errorD);
            Log.d("PositionPC", "correctionP: " + (P * errorP));
            Log.d("PositionIC", "correctionI: " + (I * errorI));
            Log.d("PositionDC", "correctionD: " + (D * errorD));
            Log.d("PositionV", "position: " + currentPosition);
            Log.d("PositionC", "power: " + correction);
            Log.d("PositionT", "time: " + timeChange);
            Log.d("PositionPL", "lastErrorP: " + lastErrorP);
            if (currentPosition / target > 1) {
                Log.d("PositionE", "precision: " + (100 * ((currentPosition / target) - 1)));
            } else {
                Log.d("PositionE", "precision: " + (100 * currentPosition / target));
            }
            telemetry.update();
        }

        wheel.setPower(0);
    }

    private double calculateErrorP(double target,double current){
        return target - current;
    }

    private double calculateErrorD(double current, double last, double time) {
        return (current - last) / time;
    }

    private double calculateErrorI(double current, double last, double time) {
        return Range.clip((last + (current * time)),-50000,50000);
    }
}
