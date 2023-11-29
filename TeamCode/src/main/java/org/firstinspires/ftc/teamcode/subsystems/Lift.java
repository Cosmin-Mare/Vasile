package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Lift {
    public DcMotorEx ascending;
    public DcMotorEx descending;

    public static double Kp = 0.002;
    public static double Ki = 0.00002;
    public static double Kd = 0.002;
    public static double Kg = 0.064;
    public static double lastError = 0;

    public static double integralSum = 0;
    public static double a = 1; // a can be anything from 0 < a < 1
    public static double previousEstimate = 0;
    public static double currentEstimate = 0;
    double lastReference;
    public static final double max_amount = 1000; //TODO: Find max amount
    public static final double min_amount = 0;
    Telemetry telemetry;
    ElapsedTime time;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime time) {
        ascending = hardwareMap.get(DcMotorEx.class, "ascending");
        descending = hardwareMap.get(DcMotorEx.class, "descending");
        this.telemetry = telemetry;
        this.time = time;
    }

    public void ascend(double amount) {
        PIDF(amount, telemetry, time);
    }

    public void descend(double amount) {
        PIDF(-amount, telemetry, time);
    }

    public double P(double reference, Telemetry telemetry) {
        double position = ascending.getCurrentPosition();

        double error = reference - position;

        double out = Kp * error;

        telemetry.addData("reference", reference);
        telemetry.addData("position", position);
        telemetry.addData("error", error);
        telemetry.update();

        return out;
    }

    public double PI(double reference, Telemetry telemetry, ElapsedTime time) {
        double position = ascending.getCurrentPosition();

        double error = reference - position;
        integralSum = integralSum + (error * time.seconds());

        if (reference != lastReference) integralSum = 0;

        lastReference = reference;
        double out = Kp * error + Ki * integralSum;

        telemetry.addData("reference", reference);
        telemetry.addData("position", position);
        telemetry.addData("error", error);
        telemetry.addData("integral sum", integralSum);
        telemetry.addData("out", out);
        telemetry.update();

        return out;
    }

    public double PID(double reference, Telemetry telemetry, ElapsedTime time) {
        double position = ascending.getCurrentPosition();

        double error = reference - position;

        double errorChange = (error - lastError);

        currentEstimate = (a * previousEstimate) + (1 - a) * errorChange;
        previousEstimate = currentEstimate;

        double derivative = currentEstimate / time.seconds();

        integralSum = integralSum + (error * time.seconds());

        if (reference != lastReference) integralSum = 0;

        lastError = error;

        lastReference = reference;

        time.reset();

        double out = Kp * error + Ki * integralSum + Kd * derivative;

        telemetry.addData("reference", reference);
        telemetry.addData("position", position);
        telemetry.addData("error", error);
        telemetry.addData("error change", errorChange);
        telemetry.addData("derivative", derivative);
        telemetry.addData("integral sum", integralSum);
        telemetry.addData("out", out);
        telemetry.update();

        return out;
    }

    public double PIDF(double reference, Telemetry telemetry, ElapsedTime time) {
        double position = ascending.getCurrentPosition();

        double error = reference - position;

        double errorChange = (error - lastError);

        currentEstimate = (a * previousEstimate) + (1 - a) * errorChange;
        previousEstimate = currentEstimate;

        double derivative = currentEstimate / time.seconds();

        integralSum = integralSum + (error * time.seconds());

        if (reference != lastReference) integralSum = 0;

        lastError = error;

        lastReference = reference;

        time.reset();

        double out = Kp * error + Ki * integralSum + Kd * derivative + Kg;

        if (position < min_amount - 10 || position > max_amount + 10) out = 0;
        telemetry.addData("reference", reference);
        telemetry.addData("position", position);
        telemetry.addData("error", error);
        telemetry.addData("error change", errorChange);
        telemetry.addData("derivative", derivative);
        telemetry.addData("integral sum", integralSum);
        telemetry.addData("out", out);
        telemetry.update();

        return out;
    }

    public void move_set_pos(int reference, Telemetry telemetry) {
        double position = descending.getCurrentPosition();

        descending.setTargetPosition(reference);
        descending.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        descending.setPower(1);

        telemetry.addData("reference", reference);
        telemetry.addData("position", position);
        telemetry.addData("error", reference - position);
        telemetry.update();
    }
}
