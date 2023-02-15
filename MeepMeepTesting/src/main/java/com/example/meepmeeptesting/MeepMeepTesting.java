package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);
        Pose2d scorePose = new Pose2d(-35, -10, Math.toRadians(45));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(160), Math.toRadians(160), 13.95)
                .setDimensions(13, 18)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(90)))
                                .addTemporalMarker(() -> {

                                })
                                .splineToSplineHeading(new Pose2d(-35, -10, Math.toRadians(45)), Math.toRadians(85))
                                .addTemporalMarker(() -> {

                                })
                                .waitSeconds(0.45)
                                .addTemporalMarker(() -> {

                                })
                                .splineToLinearHeading(new Pose2d(-40, -12, Math.toRadians(180)), Math.toRadians(180))
                                .forward(20)
                                .addTemporalMarker(() -> {

                                })
                                .waitSeconds(0.45)
                                .addTemporalMarker(() -> {

                                })
                                .back(20)
                                .addTemporalMarker(() -> {

                                })
                                .splineToLinearHeading(new Pose2d(-35, -10, Math.toRadians(45)), Math.toRadians(85))
                                .addTemporalMarker(() -> {

                                })
                                .waitSeconds(0.45)
                                .addTemporalMarker(() -> {

                                })
                                .splineToLinearHeading(new Pose2d(-40, -12, Math.toRadians(180)), Math.toRadians(180))
                                .forward(20)
                                .addTemporalMarker(() -> {

                                })
                                .waitSeconds(0.45)
                                .addTemporalMarker(() -> {

                                })
                                .back(20)
                                .addTemporalMarker(() -> {

                                })
                                .splineToLinearHeading(new Pose2d(-35, -10, Math.toRadians(45)), Math.toRadians(85))
                                .addTemporalMarker(() -> {

                                })
                                .waitSeconds(0.45)
                                .addTemporalMarker(() -> {

                                })
                                .splineToLinearHeading(new Pose2d(-40, -12, Math.toRadians(180)), Math.toRadians(180))
                                .forward(20)
                                .addTemporalMarker(() -> {

                                })
                                .waitSeconds(0.45)
                                .addTemporalMarker(() -> {

                                })
                                .back(20)
                                .addTemporalMarker(() -> {

                                })
                                .splineToLinearHeading(new Pose2d(-35, -10, Math.toRadians(45)), Math.toRadians(85))
                                .addTemporalMarker(() -> {

                                })
                                .waitSeconds(0.45)
                                .addTemporalMarker(() -> {

                                })
                                .splineToLinearHeading(new Pose2d(-40, -12, Math.toRadians(180)), Math.toRadians(180))
                                .forward(20)
                                .addTemporalMarker(() -> {

                                })
                                .waitSeconds(0.45)
                                .addTemporalMarker(() -> {

                                })
                                .back(20)
                                .addTemporalMarker(() -> {

                                })
                                .splineToLinearHeading(new Pose2d(-35, -10, Math.toRadians(45)), Math.toRadians(85))
                                .addTemporalMarker(() -> {

                                })
                                .waitSeconds(0.45)
                                .addTemporalMarker(() -> {

                                })
                                .splineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(90)), Math.toRadians(90))
                                .strafeLeft(24)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}