package org.firstinspires.ftc.teamcode.customdrive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.roboracers.topgear.geometry.Pose2d;
import com.roboracers.topgear.localization.Localizer;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class TestOpticalOdometery implements Localizer {

    Pose2d poseEstimate;

    SparkFunOTOS odo;

    public TestOpticalOdometery(HardwareMap hardwareMap) {

        odo = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        odo.setLinearUnit(DistanceUnit.INCH);
        odo.setAngularUnit(AngleUnit.RADIANS);
        odo.begin();
    }


    /**
     * Current robot pose estimate.
     */
    @Override
    public Pose2d getPoseEstimate() {
        return new Pose2d(
                odo.getPosition().x,
                odo.getPosition().y,
                odo.getPosition().h
        );
    }

    @Override
    public void setPoseEstimate(Pose2d pose) {
        odo.setPosition(new SparkFunOTOS.Pose2D(
                pose.getX(),
                pose.getY(),
                pose.getHeading()
        ));

    }

    /**
     * Current robot pose velocity (optional)
     */
    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    /**
     * Completes a single localization update.
     */
    @Override
    public void update() {

    }
}