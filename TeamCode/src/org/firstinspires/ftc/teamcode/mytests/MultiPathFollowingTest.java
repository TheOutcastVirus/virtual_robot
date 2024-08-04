package org.firstinspires.ftc.teamcode.mytests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.roboracers.topgear.geometry.Pose2d;
import com.roboracers.topgear.geometry.Vector2d;
import com.roboracers.topgear.planner.CurveBuilder;
import com.roboracers.topgear.planner.ParametricPath;

import org.firstinspires.ftc.teamcode.customdrive.CustomMecanumDrive;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
@TeleOp(name = " Multi Path Following Test", group = "1111111")
public class MultiPathFollowingTest extends LinearOpMode {

    public void runOpMode(){

        CustomMecanumDrive drive = new CustomMecanumDrive(hardwareMap);


        telemetry.addData("Press Start When Ready","");
        telemetry.update();

        waitForStart();

        double s = 50;

        /*
        ParametricPath multiPath = CurveBuilder.buildCurveSequence()
                .addCurve(new Vector2d(0,0), new Vector2d(s,0), Math.toRadians(45), Math.toRadians(45 + 180))
                .addCurve(new Vector2d(s,0), new Vector2d(0,s), Math.toRadians(45), Math.toRadians(0))
                .build();

         */

        ParametricPath multiPath = CurveBuilder.buildCurveSequence()
                .addCurve(new Vector2d(0,0), new Vector2d(s,s), Math.toRadians(45), Math.toRadians(45 + 180), 0)
                .addCurve(new Vector2d(s,s), new Vector2d(s*2,s*2), Math.toRadians(45), Math.toRadians(0), 0)
                .build();

        drive.setPath(multiPath);
        drive.setFollowing(true);

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
                drive.update();
            }

            telemetry.addLine(drive.follower.getDebugPacket().toString());
            System.out.println(drive.follower.getDebugPacket().toString());
            telemetry.update();
        }
    }
}
