package org.firstinspires.ftc.teamcode.mytests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.roboracers.pathfollower.follower.GuidedVectorFieldFollower;
import com.roboracers.pathfollower.geometry.Pose2d;
import com.roboracers.pathfollower.geometry.Vector2d;
import com.roboracers.pathfollower.planner.CubicBezierCurve;

import org.firstinspires.ftc.teamcode.customdrive.CustomMecanumDrive;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
@TeleOp(name = "Robot Translation Test", group = "MecanumBot")
public class RobotTranslationTest extends LinearOpMode {

    public void runOpMode(){

        CustomMecanumDrive drive = new CustomMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0,0,90));

        telemetry.addData("Press Start When Ready","");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()){

            Vector2d velocityVector = new Vector2d(0.5, 0.5);


            double theta = Math.toRadians(drive.getPoseEstimate().getHeading());

            // Compute the rotation matrix elements
            double cosTheta = Math.cos(theta);
            double sinTheta = Math.sin(theta);

            // Apply the rotation matrix to the vector
            double xRobot = cosTheta * velocityVector.getX() + sinTheta * velocityVector.getY();
            double yRobot = -sinTheta * velocityVector.getX() + cosTheta * velocityVector.getY();

            // Return the new vector in the robot's frame of reference


            Pose2d drivePower = new Pose2d(new Vector2d(xRobot, yRobot), 0);

            drive.setDrivePower(drivePower);

            drive.updatePoseEstimate();

            telemetry.addData("Pos:", drive.getPoseEstimate().toString());
            telemetry.addData("Drive Power", drivePower.toString());
            telemetry.update();
        }
    }
}
