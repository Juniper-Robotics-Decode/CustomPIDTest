package org.firstinspires.ftc.teamcode.Shooter;

import android.os.Environment;
import com.opencsv.CSVWriter;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

@TeleOp
public class ShooterPID extends LinearOpMode {

    DcMotorEx wheel;
    ElapsedTime timer = new ElapsedTime();

    double target = 2500;
    double integralSum = 0;
    double lastError = 0;

    double p = 0.01;
    double i = 0.001;
    double d = 0.0001;
    double f = 0.00015;

    int count = 0;
    boolean ready = true;
    boolean dataCollected = false;

    File logFile;

    @Override
    public void runOpMode() {
        wheel = hardwareMap.get(DcMotorEx.class, "wheel");
        wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        logFile = new File(String.format("%s/FIRST/shooterpid.csv", Environment.getExternalStorageDirectory().getAbsolutePath()));

        waitForStart();
        timer.reset();

        if (!dataCollected) {
            collectShooterData(7500);
            dataCollected = true;
        } else {
            telemetry.addData("-", "DATA COLLECTION FINISHED");
            telemetry.update();
        }
    }

    public void collectShooterData(int data) {
        ArrayList<String[]> dataArray = new ArrayList<>();
        dataArray.add(new String[]{"Time", "Target", "Actual", "Error", "Power"});
        ElapsedTime sampleTimer = new ElapsedTime();

        for (int j = 0; j < data; j++) {
            double currentvel = wheel.getVelocity();

            double error = target - currentvel;
            double dt = timer.seconds();
            timer.reset();

            if (Math.abs(error) < 100) {
                integralSum = integralSum + (error * dt);
            }

            double derivative = (error - lastError) / dt;
            lastError = error;
            double power = (p * error) + (i * integralSum) + (d * derivative) + (f * target);

            if (power > 1) power = 1;
            if (power < 0) power = 0;
            wheel.setPower(power);

            if (Math.abs(error) < 50 && ready) {
                ready = false;
                count = count + 1;
            }
            if (Math.abs(error) > 150) {
                ready = true;
            }

            dataArray.add(new String[]{
                    String.valueOf(sampleTimer.seconds()),
                    String.valueOf(target),
                    String.valueOf(currentvel),
                    String.valueOf(error),
                    String.valueOf(power)
            });

            telemetry.addData("Target", target);
            telemetry.addData("Actual", currentvel);
            telemetry.addData("Error", error);
            telemetry.addData("Count", count);
            if (Math.abs(error) < 50) {
                telemetry.addData("Fire?", "Ready");
            } else {
                telemetry.addData("Fire?", "Spinning back up to speed");
            }
            telemetry.addData("Progress", j + " / " + data);
            telemetry.update();

            if (!opModeIsActive()) break;
        }

        try (FileWriter fw = new FileWriter(logFile);
             CSVWriter writer = new CSVWriter(fw)) {
            writer.writeAll(dataArray);
        } catch (IOException e) {
            telemetry.addData("Error writing CSV", e.getMessage());
            telemetry.update();
        }
    }
}