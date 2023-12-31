package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.OutTake;

public class Robot {
    Lift lift;
    MecanumDrive drive;
    Intake intake;
    OutTake outTake;
    Launcher launcher;

    public Robot(HardwareMap hardwareMap) {
        lift = new Lift(hardwareMap);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        intake = new Intake(hardwareMap);
        outTake = new OutTake(hardwareMap);
        launcher = new Launcher(hardwareMap);
    }


}
