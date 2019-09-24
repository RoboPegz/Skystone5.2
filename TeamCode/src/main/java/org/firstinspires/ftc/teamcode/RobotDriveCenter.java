package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotDriveCenter {

    void processDriverCentricDriveMode () {
        double leftFrontAngle, rightFrontAngle, leftRearAngle, rightRearAngle;
        double gyroAngle;

        //Retrieve/scale joystick motor input
        double yTranslation =  gamepad1.left_stick_y;
        double xTranslation = -gamepad1.left_stick_x;
        double rotation     =  gamepad1.right_stick_x;

        gyroAngle = robot.headingIMU

        if (gamepad1.x) {
            //the driver presses X, then uses the left joystick to say what anfle the robot is aiming
            //button should be released before stick
            driverAngle = -Math.toDegrees( Math.atan2( -gamepad1.left_stick_x, gamepad1.left_stick_y));
        }

    }

}
