package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import android.util.Log;

@TeleOp
public class VelocityPID extends LinearOpMode {
    DcMotorEx wheel;
    double P = 0.007; //increase if its taking too long to get to target velocity
    double I = 0.018; //change if its not getting close enough to the target velocity
    double D = 0.00013; //change if its jumpy
    double F = 1 / 245; //dont touch
    double targetVelocity = 95;
    double lastErrorP = 0;
    double lastErrorI = 0;
    @Override
    public void runOpMode() {
        wheel = hardwareMap.get(DcMotorEx.class, "wheel");
        wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        double lastTime = getRuntime();
        while (opModeIsActive()) {
            double currentTime = getRuntime();
            double timeChange  = currentTime - lastTime;
            if(timeChange <= 0){
                timeChange = 0.004;
            }
            double currentVelocity = wheel.getVelocity(DEGREES);
            double errorP = calculateErrorP(currentVelocity);
            double errorD = calculateErrorD(errorP,lastErrorP,timeChange);
            double errorI = calculateErrorI(errorP,lastErrorI,timeChange);
            double correction = (P * errorP) + (I * errorI) + (D * errorD) + (F * targetVelocity);
            wheel.setPower(Range.clip(correction,-1,1));
            lastErrorP = errorP;
            lastErrorI = errorI;
            lastTime = currentTime;

            telemetry.addData("errorP: ", errorP);
            telemetry.addData("lastErrorP", lastErrorP);
            telemetry.addData("errorI: ", errorI);
            telemetry.addData("errorD: ", errorD);
            Log.d( "correctionP: ", String.valueOf(P * errorP));
            Log.d("correctionI: ", String.valueOf(I * errorI));
            Log.d("correctionD: ", String.valueOf(D * errorD));
//            wheel.setPower(1);
            telemetry.addData("velocity: ", currentVelocity);
            telemetry.addData( "correction: ", correction);
            telemetry.addData("time: ", timeChange);
            Log.d("velocityP", "errorP: " + errorP);
            Log.d("velocityI", "errorI: " + errorI);
            Log.d("velocityD", "errorD: " + errorD);
            Log.d("velocityPC", "correctionP: " + (P * errorP));
            Log.d("velocityIC", "correctionI: " + (I * errorI));
            Log.d("velocityDC", "correctionD: " + (D * errorD));
            Log.d("velocityV", "velocity: " + currentVelocity);
            Log.d("velocityC", "power: " + correction);
            Log.d("velocityT","time: " + timeChange);
            Log.d("velocityPL","lastErrorP: " + lastErrorP);
            Log.d("velocityE","precision: " + (100 * currentVelocity/targetVelocity));
            telemetry.update();

        }
        if (isStopRequested()) {
            wheel.setPower(0);
        }
    }
    public double calculateErrorP(double current){
        double errorP = targetVelocity - current;
        return errorP;
    }

    public double calculateErrorD(double current, double last, double time) {
        double errorD = (current - last) / time;
        return errorD;
    }

    public double calculateErrorI(double current, double last, double time) {
        double errorI = last + (current * time);
        return Range.clip(errorI,-50000,50000);
    }
}
