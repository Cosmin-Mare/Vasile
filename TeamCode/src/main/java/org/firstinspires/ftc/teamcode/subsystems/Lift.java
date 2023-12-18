package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Lift {
    public Servo tureta;
    public Servo unghiTureta;
    public Servo avion;



//    public DcMotorEx ascending;
//    public DcMotorEx descending;

    private final int MIN_POS_DESC = 0;
    private final int MAX_POS_ASC = 999999999;

    private final int MAX_POS_DESC = 999999999;
    private final int MIN_POS_ASC = 0;

    public Lift(@NonNull HardwareMap hardwareMap) {
//        ascending = hardwareMap.get(DcMotorEx.class, "ascending");
//        descending = hardwareMap.get(DcMotorEx.class, "descending");
//
//        ascending.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        descending.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        ascending.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        descending.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        tureta = hardwareMap.get(Servo.class, "tureta");
//        tureta.setPosition(0);
//
//        unghiTureta = hardwareMap.get(Servo.class, "unghiTureta");
//        unghiTureta.setPosition(0);

        avion = hardwareMap.get(Servo.class, "avion");
        avion.setPosition(0);
    }
//    public void ascend() {
//        if(ascending.getCurrentPosition() >= MIN_POS_ASC && ascending.getCurrentPosition() <= MAX_POS_ASC &&
//        descending.getCurrentPosition() >= MIN_POS_DESC && descending.getCurrentPosition() <= MAX_POS_DESC) {
//            ascending.setPower(-1);
//            descending.setTargetPosition(ascending.getCurrentPosition());
//            descending.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            descending.setPower(1);
//        }
//    }
//    public void descend() {
//        if(ascending.getCurrentPosition() >= MIN_POS_ASC && ascending.getCurrentPosition() <= MAX_POS_ASC &&
//                descending.getCurrentPosition() >= MIN_POS_DESC && descending.getCurrentPosition() <= MAX_POS_DESC) {
//            ascending.setPower(1);
//            descending.setTargetPosition(ascending.getCurrentPosition());
//            descending.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            descending.setPower(1);
//        }
//    }
}
