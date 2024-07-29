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
@TeleOp(name = "Mecanum Drive Test", group = "MecanumBot")
public class CustomMecTest extends LinearOpMode {

    public void runOpMode(){

        CustomMecanumDrive drive = new CustomMecanumDrive(hardwareMap);


        GuidedVectorFieldFollower follower = new GuidedVectorFieldFollower(0.5);

        drive.setPoseEstimate(new Pose2d(35.69,27.82,Math.toRadians(90)));

        follower.setPath(new CubicBezierCurve(
                new Vector2d(0,0), new Vector2d(72,0),
                new Vector2d(0,72), new Vector2d(72,72)));


        telemetry.addData("Press Start When Ready","");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()){

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            gamepad1.right_stick_x
                    )
            );

            telemetry.addData("left stick y", gamepad1.left_stick_y);
            telemetry.addData("left stick x", gamepad1.left_stick_x);
            telemetry.addData("right stick x", gamepad1.right_stick_x);
            telemetry.addLine(drive.getPoseEstimate().toString());
            telemetry.addData("To be drive power", follower.getDriveVelocity(new Pose2d(drive.getPoseEstimate().vec(), 0)).toString());
            telemetry.update();
        }
    }
}
