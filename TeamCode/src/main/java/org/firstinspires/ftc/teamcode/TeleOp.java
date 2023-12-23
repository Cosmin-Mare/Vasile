package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends OpMode {
    enum IntakeState {
        INTAKE,
        OUTTAKE,
        STOP
    }

    Robot robot;
    ElapsedTime time;
    IntakeState intakeState = IntakeState.STOP;
    MultipleTelemetry tele;
    @Override
    public void init() {
        time = new ElapsedTime();
        tele = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        robot = new Robot(hardwareMap);

    }
    @Override
    public void loop() {
        robot.drive.moveRobot(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

        if (gamepad1.right_stick_button) intakeState = (intakeState == IntakeState.STOP) ? IntakeState.INTAKE : IntakeState.STOP;
        if (gamepad1.left_stick_button) intakeState = (intakeState == IntakeState.STOP) ? IntakeState.OUTTAKE : IntakeState.STOP;

        switch (intakeState) {
            case INTAKE: robot.intake.takeIn(); break;
            case OUTTAKE: robot.intake.takeOut(); break;
            case STOP: robot.intake.stop(); break;
        }

        if (gamepad2.a) robot.launcher.airplaneIn();

        if (gamepad2.x) robot.launcher.prepareLaunch();
        if (gamepad2.y) robot.launcher.launch();

        if (gamepad1.right_bumper) robot.outTake.catchPixels();
        if (gamepad1.left_bumper) robot.outTake.releasePixels();

        if (gamepad1.x) robot.intake.resIntake();
        if (gamepad1.y) robot.intake.lowIntake();

        if (gamepad1.dpad_down) robot.outTake.prepareCatch();
        if (gamepad1.dpad_up) robot.outTake.prepareRelease();

        if (gamepad1.dpad_left) {
            robot.lift.descend();
        } else if (gamepad1.dpad_right) {
            robot.lift.ascend();
        } else {
            robot.lift.stop();
        }

        if(gamepad1.right_trigger > 0.5) {
            robot.lift.descending.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lift.descending.setPower(1);
        }
        if(gamepad1.left_trigger > 0.5) {
            robot.lift.descending.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lift.descending.setPower(-1);
        }
//
        robot.drive.updatePoseEstimate();
        tele.addData("lift asc position and power", robot.lift.ascending.getCurrentPosition()+" and "+robot.lift.ascending.getPower());
        tele.addData("x", robot.drive.pose.position.x);
        tele.addData("y", robot.drive.pose.position.y);
        tele.addData("heading", robot.drive.pose.heading);
        tele.addData("voltage" , robot.lift.ascending.getCurrent(CurrentUnit.AMPS));
        tele.update();
    }
}
