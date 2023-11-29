package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public DcMotor intake;
    public Servo stangaIntake;
    public Servo dreaptaIntake;
    public Intake(HardwareMap hardwareMap){
        intake = hardwareMap.get(DcMotor.class, "intake");
        stangaIntake = hardwareMap.get(Servo.class, "stangaIntake");
        dreaptaIntake = hardwareMap.get(Servo.class, "dreaptaIntake");
        stangaIntake.setPosition(0.48);
    }

    public void takeIn(){
        intake.setPower(1);
    }
    public void takeOut(){
        intake.setPower(-1);
    }
    public void stop(){
        intake.setPower(0);
    }
}
