package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

enum IntakeState {
    INTAKE,
    OUTTAKE,
    STOP
}

enum OutTakeState {
    ASCEND,
    DESCEND,
    STOP
}

@TeleOp(name = "TestTeleOp")
public class TestTeleOp extends OpMode {
    Robot robot;
    ElapsedTime time;
    IntakeState intakeState = IntakeState.STOP;
    OutTakeState outTakeState = OutTakeState.STOP;

    @Override
    public void init() {
        time = new ElapsedTime();
        robot = new Robot(hardwareMap, telemetry, time);
//        robot.lift.move_set_pos(30000, telemetry);
    }

    @Override
    public void loop() {
        telemetry.addData("lift position", robot.lift.descending.getCurrentPosition());
        robot.drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
        ));
        if (gamepad1.right_stick_button) {
            if (intakeState == IntakeState.STOP)
                intakeState = IntakeState.INTAKE;
            else {
                intakeState = IntakeState.STOP;
            }
        } else {
            if (gamepad1.left_stick_button) {
                if (intakeState == IntakeState.STOP) {
                    intakeState = IntakeState.OUTTAKE;
                } else {
                    intakeState = IntakeState.STOP;
                }
            }
        }
        if (gamepad1.right_bumper) {
            robot.outTake.catchPixels();
        } else {
            if (gamepad1.left_bumper) {
                robot.outTake.releasePixels();
            }
        }
        switch (intakeState) {
            case INTAKE:
                robot.intake.takeIn();
                break;
            case OUTTAKE:
                robot.intake.takeOut();
                break;
            case STOP:
                robot.intake.stop();
                break;
        }
        if (gamepad1.x) {
            robot.intake.dreaptaIntake.setPosition(0.13);
            robot.intake.stangaIntake.setPosition(0.87);
        }
        if (gamepad1.y) {
            robot.intake.dreaptaIntake.setPosition(0.6);
            robot.intake.stangaIntake.setPosition(0.4);
        }

        if (gamepad1.dpad_down) {
            robot.outTake.prepareCatch();
        } else if (gamepad1.dpad_up) {
            robot.outTake.prepareRelease();
        }

        robot.outTake.brat.setPower(-gamepad2.left_stick_y);
        robot.lift.descending.setPower(-gamepad2.right_stick_y);

        telemetry.addData("brat power", robot.outTake.brat.getPower());

        telemetry.addData("brat position", robot.outTake.brat.getCurrentPosition());


        if (gamepad1.right_trigger > 0.5) {
            telemetry.addData("AAAAAAAAAAAA", "AAAAAAAAAAAAAAAAAa");
            telemetry.addData("robot lift asceding power and position", robot.lift.ascending.getCurrentPosition() + " and " + robot.lift.ascending.getPower());
            robot.lift.ascending.setPower(-gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > 0.5) {
            telemetry.addData("AAAAAAAAAAAA", "AAAAAAAAAAAAAAAAAa");
            telemetry.addData("robot lift asceding power and position", robot.lift.ascending.getCurrentPosition() + " and " + robot.lift.ascending.getPower());
            robot.lift.ascending.setPower(gamepad1.left_trigger);
        } else {
            robot.lift.ascending.setPower(0);
        }

        robot.drive.updatePoseEstimate();

        telemetry.addData("x", robot.drive.pose.position.x);
        telemetry.addData("y", robot.drive.pose.position.y);
        telemetry.addData("heading", robot.drive.pose.heading);
        telemetry.update();
    }
}
