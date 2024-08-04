package org.firstinspires.ftc.teamcode.mytests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.roboracers.topgear.follower.GuidedVectorFieldFollower;
import com.roboracers.topgear.geometry.Pose2d;
import com.roboracers.topgear.geometry.Vector2d;
import com.roboracers.topgear.planner.CubicBezierCurve;

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
