package org.firstinspires.ftc.teamcode.mytests;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.roboracers.pathfollower.follower.GuidedVectorFieldFollower;
import com.roboracers.pathfollower.geometry.Pose2d;
import com.roboracers.pathfollower.geometry.Vector2d;
import com.roboracers.pathfollower.planner.CubicBezierCurve;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.customdrive.CustomMecanumDrive;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
@TeleOp(name = "Path Following Test", group = "1111")
public class PathFollowingTest extends LinearOpMode {

    public void runOpMode(){

        CustomMecanumDrive drive = new CustomMecanumDrive(hardwareMap);

        GuidedVectorFieldFollower follower = new GuidedVectorFieldFollower(1, 1);

        drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(90)));

        follower.setPath(new CubicBezierCurve(
                        new Vector2d(0,0), new Vector2d(24,24),
                         new Vector2d(-24,24), new Vector2d(0,48)));

        telemetry.addData("Press Start When Ready","");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()){

            if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0 || gamepad1.right_stick_x != 0) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                gamepad1.right_stick_x
                        )
                );
            } else {


                Pose2d drivePower = follower.getDriveVelocity(drive.getPoseEstimate());

                drive.setDrivePower(drivePower);
                telemetry.addData("Pos:", drive.getPoseEstimate().toString());
                telemetry.addData("Drive Power", drivePower.toString());
            }


            telemetry.update();
        }
    }
}
