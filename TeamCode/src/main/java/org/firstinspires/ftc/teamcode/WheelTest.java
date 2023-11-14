package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "WheelTest")
public class WheelTest extends OpMode {
    MecanumDrive drive;
    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    @Override
    public void loop() {
        if(gamepad1.x){
            drive.leftFront.setPower(1);
        }
        if(gamepad1.y){
            drive.rightFront.setPower(1);
        }
        if(gamepad1.b){
            drive.rightBack.setPower(1);
        }
        if(gamepad1.a){
            drive.leftBack.setPower(1);
        }
    }
}
