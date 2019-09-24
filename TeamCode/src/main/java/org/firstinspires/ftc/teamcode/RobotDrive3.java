package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp public class RobotDrive3 extends OpMode {
    DcMotor UpleftDrive;
    DcMotor UprightDrive;
    DcMotor DownleftDrive;
    DcMotor DownrightDrive;


    @Override
    public void init() {
    UpleftDrive = hardwareMap.dcMotor.get("uld");
    UprightDrive = hardwareMap.dcMotor.get("urd");
    DownleftDrive = hardwareMap.dcMotor.get("dld");
    DownrightDrive = hardwareMap.dcMotor.get("drd");

    UpleftDrive.setPower(0);
    UprightDrive.setPower(0);
    DownleftDrive.setPower(0);
    DownrightDrive.setPower(0);
    }


    void processDriverCentricDriveMode () {
        double leftFrontAngle, rightFrontAngle, leftRearAngle, rightRearAngle;
        double gyroAngle;

        //Retrieve/scale joystick motor input
        double yTranslation =  gamepad1.left_stick_y;
        double xTranslation = -gamepad1.left_stick_x;
        double rotation     =  gamepad1.right_stick_x;

        gyroAngle = robot.headingIMU ();

        if (gamepad1.x) {
            //the driver presses X, then uses the left joystick to say what anfle the robot is aiming
            //button should be released before stick
            driverAngle = -Math.toDegrees( Math.atan2( -gamepad1.left_stick_x, gamepad1.left_stick_y));

            if (driverAngle < 0) {
                driverAngle += 360.0;
            }
            driverAngle -= gyroAngle;
            xTranslation = 0.0;
            yTranslation = 0.0;
            rotation     = 0.0;
        }

        //Adjust new gyro angle for the driver reference angle
        gyroAngle += driverAngle;

        //compute motor angles (both left and right motors are defined FORWARD)
        rightFrontAngle = Math.toRadians( gyroAngle + 315);
        leftFrontAngle  = Math.toRadians( gyroAngle + 45);
        rightRearAngle  = Math.toRadians( gyroAngle + 225);
        leftRearAngle   = Math.toRadians( gyroAngle + 135);

        frontRight = (yTranslation * Math.sin(rightFrontAngle) + xTranslation * Math.cos(rightFrontAngle))/Math.sqrt(2) + rotation;
        frontLeft  = (yTranslation * Math.sin(leftFrontAngle)  + xTranslation * Math.cos(leftFrontAngle))/Math.sqrt(2)  + rotation;
        backRight  = (yTranslation * Math.sin(rightRearAngle)  + xTranslation * Math.cos(rightRearAngle))/Math.sqrt(2)  + rotation;
        backLeft   = (yTranslation * Math.sin(leftRearAngle)   + xTranslation * Math.cos(leftRearAngle))/Math.sqrt(2)   + rotation;

        //Normalize the values so none exceed +/- 1.0
        maxPower = Math.max( Math.max( Math.abs(backLeft), Math.abs(backRight)  ),  Math.max(frontLeft), Math.abs(frontRight) );

        if (maxPower > 1.0) {
            backLeft   /= maxPower;
            backRight  /= maxPower;
            frontLeft  /= maxPower;
            frontRight /= maxPower;
        }

        //Update motor power settings:
        robot.frontLeftMotor.setPower( frontLeft );
        robot.frontRightMotor.setPower( frontRight );
        robot.backLeftMotor.setPower( backLeft );
        robot.backRightMotor.setPower( backRight );
    }

    public double headingIMU() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES );

        double heading = (double)angles.firstAngle;
        return heading;

    } //headingIMU


    @Override
    public void loop() {
        UpleftDrive.setPower(0);
        UprightDrive.setPower(0);
        DownleftDrive.setPower(0);
        DownrightDrive.setPower(0);


        if (gamepad1.dpad_down) {
            UpleftDrive.setPower(1);
            UprightDrive.setPower(-1);
            DownrightDrive.setPower(-1);
            DownleftDrive.setPower(1);
        }
        else {
            UprightDrive.setPower(0);
            UprightDrive.setPower(0);
            DownrightDrive.setPower(0);
            DownleftDrive.setPower(0);
        }
        if (gamepad1.dpad_left) {
            UpleftDrive.setPower(1);
            UprightDrive.setPower(1);
            DownleftDrive.setPower(-1);
            DownrightDrive.setPower(-1);
        }
        else {
            UprightDrive.setPower(0);
            UprightDrive.setPower(0);
            DownrightDrive.setPower(0);
            DownleftDrive.setPower(0);
        }
        if(gamepad1.dpad_right) {
            UpleftDrive.setPower(-1);
            UprightDrive.setPower(-1);
            DownleftDrive.setPower(1);
            DownrightDrive.setPower(1);
        }
        else {
            UprightDrive.setPower(0);
            UprightDrive.setPower(0);
            DownrightDrive.setPower(0);
            DownleftDrive.setPower(0);
        }
        if(gamepad1.dpad_up) {
            UpleftDrive.setPower(-1);
            UprightDrive.setPower(1);
            DownleftDrive.setPower(-1);
            DownrightDrive.setPower(1);
        }
        else {
            UprightDrive.setPower(0);
            UprightDrive.setPower(0);
            DownrightDrive.setPower(0);
            DownleftDrive.setPower(0);
        }

            if (gamepad1.left_bumper) {
                UpleftDrive.setPower(1);
                UprightDrive.setPower(1);
                DownleftDrive.setPower(1);
                DownrightDrive.setPower(1);
            }
            else {
                UprightDrive.setPower(0);
                UprightDrive.setPower(0);
                DownrightDrive.setPower(0);
                DownleftDrive.setPower(0);
            }
            if (gamepad1.right_bumper) {
                UpleftDrive.setPower(-1);
                UprightDrive.setPower(-1);
                DownleftDrive.setPower(-1);
                DownrightDrive.setPower(-1);
            }
            else {
                UprightDrive.setPower(0);
                UprightDrive.setPower(0);
                DownrightDrive.setPower(0);
                DownleftDrive.setPower(0);
            }




    }
}
