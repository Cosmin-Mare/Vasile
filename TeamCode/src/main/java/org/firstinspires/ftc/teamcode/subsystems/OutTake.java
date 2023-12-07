package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OutTake {
    public DcMotorEx brat;
    Servo dreaptaGripper;
    Servo stangaGripper;
    Servo servoBrat;
    public OutTake(HardwareMap hardwareMap){
        brat = hardwareMap.get(DcMotorEx.class, "brat");
        dreaptaGripper = hardwareMap.get(Servo.class, "dreaptaGripper");
        stangaGripper = hardwareMap.get(Servo.class, "stangaGripper");
        servoBrat = hardwareMap.get(Servo.class, "servoBrat");
        brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void prepareCatch(){
        servoBrat.setPosition(0.5);
        releasePixels();

    }

    public void prepareRelease(){
        //TODO: Set position motor brat
        servoBrat.setPosition(1);
    }

    public void catchPixels(){
        dreaptaGripper.setPosition(0.55);
        stangaGripper.setPosition(0.1);
    }

    public void releasePixelRight(){
        dreaptaGripper.setPosition(1);
    }
    public void releasePixelLeft(){
        stangaGripper.setPosition(0);
    }
    public void releasePixels(){
        stangaGripper.setPosition(0);
        dreaptaGripper.setPosition(0.8);
    }
}
