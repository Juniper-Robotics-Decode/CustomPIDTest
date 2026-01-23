package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimitSwitchHome {
    public DcMotorEx wheel;
    public AnalogInput ls;

    public LimitSwitchHome(HardwareMap hardwareMap) {

        wheel = hardwareMap.get(DcMotorEx.class, "wheel");
        ls = hardwareMap.get(AnalogInput.class, "ls");

        wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (ls.getVoltage() > 1.0) {
            wheel.setPower(0.3);
        }
        wheel.setPower(0);
    }
}