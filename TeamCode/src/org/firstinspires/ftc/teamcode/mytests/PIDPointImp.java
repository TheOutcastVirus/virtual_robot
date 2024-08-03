package org.firstinspires.ftc.teamcode.mytests;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.roboracers.pathfollower.controls.PIDController;
import com.roboracers.pathfollower.geometry.Pose2d;
import com.roboracers.pathfollower.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.customdrive.CustomMecanumDrive;


@Config
@TeleOp(name = "PID to Point", group = "16481-Centerstage")
public class PIDPointImp extends LinearOpMode {

    CustomMecanumDrive drive;
    public static double xkP = 1;
    public static double xkI = 0;
    public static double xkD = 0;
    public static double ykP = 1;
    public static double ykI = 0;
    public static double ykD = 0;
    public static double hkP = 1;
    public static double hkI = 0;
    public static double hkD = 0;

    public static double xTarget = -25;
    public static double yTarget = 50;
    public static double headingTarget = 45;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new CustomMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(90)));


        waitForStart();

        while (!isStopRequested()) {

            PIDController xPID =  new PIDController(xkP,xkI,xkD);
            PIDController yPID =  new PIDController(ykP,ykI,ykD);
            PIDController headingPID =  new PIDController(hkP,hkI,hkD);

            xPID.setSetpoint(xTarget);
            yPID.setSetpoint(yTarget);
            headingPID.setSetpoint(Math.toRadians(headingTarget));

            Pose2d currentPose = drive.getPoseEstimate();

            Vector2d translationPowers = new Vector2d(
                    xPID.update(currentPose.getX()),
                    yPID.update(currentPose.getY())
            ).rotated(-currentPose.getHeading());

            drive.setWeightedDrivePower(
                    new Pose2d(
                            translationPowers,
                            headingPID.update(currentPose.getHeading())
                    )
            );

            // Update all state machines
            drive.updatePoseEstimate();

            // Telemetry
            telemetry.addLine("\uD83C\uDFCE PID to Point Implementation");
            telemetry.addData("trans x",  translationPowers.getX());
            telemetry.addData("trans y",  translationPowers.getY());
            telemetry.addData("heading",headingPID.update(currentPose.getHeading()) );
            telemetry.update();
        }
    }
}