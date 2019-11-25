package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class Gyros extends LinearOpMode{
    Hardware robot = new Hardware();
    public Orientation lastAngles = new Orientation();
    public double globalAngle;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !robot.imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        waitForStart();

        rotate(90);
        rotate(-90);

        telemetry.addData("Turned: ", "%7f", getAngle()); telemetry.update();
        sleep(2000);
    }

    private void resetAngle() {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;

    }

    private double getAngle() {
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public void rotate(double degrees) {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        double k = 0.025;

        double error = (degrees - getAngle())*k;

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        //if (degrees < 0) {
           // turn right.
        leftPower = -error;
        rightPower = error;
        //}
       // else if (degrees > 0) {
            // turn left.
        //leftPower = -power;
        //rightPower = power;
        //}
        //else return;

        // set power to rotate.
        robot.leftFrontDrive.setPower(leftPower);
        robot.leftRearDrive.setPower(leftPower);
        robot.rightFrontDrive.setPower(rightPower);
        robot.rightRearDrive.setPower(rightPower);



        // rotate until turn is completed.
        /*if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }*/

        // left turn.

        runtime.reset();
        while (opModeIsActive() && Math.abs(getAngle()-degrees) <= 1.5) {
            error = (degrees - getAngle())*k;

            leftPower = -error;
            rightPower = error;

            robot.leftFrontDrive.setPower(leftPower);
            robot.leftRearDrive.setPower(leftPower);
            robot.rightFrontDrive.setPower(rightPower);
            robot.rightRearDrive.setPower(rightPower);

            telemetry.addData("Angle: ", "%7f", getAngle());
            telemetry.addData("Power: ", "%7f", error);
            telemetry.update();
        }

        // turn the motors off.
        robot.leftFrontDrive.setPower(0);
        robot.leftRearDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightRearDrive.setPower(0);
        telemetry.addData("Angle: ", "%7f", getAngle()); telemetry.update();

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
}