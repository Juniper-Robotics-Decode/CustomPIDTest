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
import java.util.ArrayList;

@TeleOp(name="ShooterPID")
public class ShooterPID extends LinearOpMode {

    DcMotorEx wheel;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime runtime = new ElapsedTime();

    public static double p = 0.0015;
    public static double i = 0.0035;
    public static double d = 0.0004;

    double target = 2500;
    double integralSum = 0;
    double lastError = 0;
    double CPM = 28;

    @Override
    public void runOpMode() {
        wheel = hardwareMap.get(DcMotorEx.class, "wheel");
        wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ArrayList<String[]> dataLog = new ArrayList<>();
        dataLog.add(new String[]{"Time", "Target", "Actual", "Error", "Power"});

        waitForStart();
        timer.reset();
        runtime.reset();

        for (int j = 0; j < 5000 && opModeIsActive(); j++) {

            if (gamepad1.cross) {
                integralSum = 0;
                lastError = 0;
            }
            if (gamepad1.dpad_up) target += 5;
            if (gamepad1.dpad_down) target -= 5;

            double actualTPS = wheel.getVelocity();
            double actualRPM = (actualTPS / CPM) * 60;
            double error = target - actualRPM;
            double dt = timer.seconds();
            if (dt <= 0) dt = 0.001;
            timer.reset();

            if (Math.abs(error) < 800) {
                integralSum += (error * dt);
            }

            if (integralSum > 2000) integralSum = 2000;
            if (integralSum < -2000) integralSum = -2000;

            double derivative = (error - lastError) / dt;
            lastError = error;

            double power = (p * error) + (i * integralSum) + (d * derivative);

            if (power > 1) power = 1;
            if (power < -1) power = -1;
            wheel.setPower(power);

            dataLog.add(new String[]{
                    String.valueOf(runtime.seconds()),
                    String.valueOf(target),
                    String.valueOf(actualRPM),
                    String.valueOf(error),
                    String.valueOf(power)
            });

            telemetry.addData("Status", "Recording " + j + "/5000");
            telemetry.addData("Target Speed", target);
            telemetry.addData("Actual Speed", actualRPM);
            telemetry.addData("Error", error);
            telemetry.addData("Power %", power * 100);
            telemetry.update();
        }

        wheel.setPower(0);
        sleep(10);
        saveCSV(dataLog);
    }

    private void saveCSV(ArrayList<String[]> data) {
        try {
            File path = new File(Environment.getExternalStorageDirectory().getAbsolutePath());
            File file = new File(path, "2500updatedtest1.csv");

            CSVWriter writer = new CSVWriter(new FileWriter(file));
            writer.writeAll(data);
            writer.close();
            telemetry.addData("Status", "Saved to " + file.getAbsolutePath());
            telemetry.update();
            sleep(2000);

        } catch (Exception e) {
            telemetry.addData("Error", e.getMessage());
            telemetry.update();
            sleep(5000);
        }
    }
}