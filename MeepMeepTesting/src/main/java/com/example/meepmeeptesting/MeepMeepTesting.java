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
                        drive.trajectorySequenceBuilder(new Pose2d(36, -63, Math.toRadians(90)))
                                .addTemporalMarker(() -> {
                                    //targetInches = 0;
                                    //gripServo.setPosition(1.0);
                                })
                                .waitSeconds(0.35)
                                .addDisplacementMarker(() -> {
                                    //targetInches = 37;
                                })
                                .splineTo(new Vector2d(36, -27), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(33.25, -11.25, Math.toRadians(135)), Math.toRadians(125))
                                .waitSeconds(0.5)
                                .addTemporalMarker(() -> {
                                    //guide.setPosition(0.0);
                                })
                                .waitSeconds(0.35)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(0.75);
                                })
                                .waitSeconds(0.25)
                                .addTemporalMarker(() -> {
                                    //guide.setPosition(0.33);
                                    //targetInches = 6;
                                })
                                .lineToSplineHeading(new Pose2d(40, -14, Math.toRadians(0)))
                                .splineToLinearHeading(new Pose2d(59, -14, Math.toRadians(0)), Math.toRadians(0))
                                .waitSeconds(0.2)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(1.0);
                                })
                                .waitSeconds(0.3)
                                .addTemporalMarker(() -> {
                                    //targetInches = 37;
                                })
                                .back(19)
                                .splineToSplineHeading(new Pose2d(33.25, -11.25, Math.toRadians(135)), Math.toRadians(150))
                                .waitSeconds(0.5)
                                .addTemporalMarker(() -> {
                                    //guide.setPosition(0.0);
                                })
                                .waitSeconds(0.35)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(0.75);
                                })
                                .waitSeconds(0.25)
                                .addTemporalMarker(() -> {
                                    //guide.setPosition(0.33);
                                    //targetInches = 5;
                                })
                                .lineToSplineHeading(new Pose2d(40, -14, Math.toRadians(0)))
                                .splineToLinearHeading(new Pose2d(59, -14, Math.toRadians(0)), Math.toRadians(0))
                                .waitSeconds(0.2)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(1.0);
                                })
                                .waitSeconds(0.3)
                                .addTemporalMarker(() -> {
                                    //targetInches = 37;
                                })
                                .back(19)
                                .splineToSplineHeading(new Pose2d(33.25, -11.25, Math.toRadians(135)), Math.toRadians(150))
                                .waitSeconds(0.5)
                                .addTemporalMarker(() -> {
                                    //guide.setPosition(0.0);
                                })
                                .waitSeconds(0.35)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(0.75);
                                })
                                .waitSeconds(0.25)
                                .addTemporalMarker(() -> {
                                    //guide.setPosition(0.33);
                                    //targetInches = 3;
                                })
                                .lineToSplineHeading(new Pose2d(40, -14, Math.toRadians(0)))
                                .splineToLinearHeading(new Pose2d(59, -14, Math.toRadians(0)), Math.toRadians(0))
                                .waitSeconds(0.2)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(1.0);
                                })
                                .waitSeconds(0.3)
                                .addTemporalMarker(() -> {
                                    //targetInches = 37;
                                })
                                .back(19)
                                .splineToSplineHeading(new Pose2d(33.25, -11.25, Math.toRadians(135)), Math.toRadians(150))
                                .waitSeconds(0.5)
                                .addTemporalMarker(() -> {
                                    //guide.setPosition(0.0);
                                })
                                .waitSeconds(0.35)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(0.75);
                                })
                                .waitSeconds(0.25)
                                .addTemporalMarker(() -> {
                                    //guide.setPosition(0.33);
                                    //targetInches = 0;
                                    //gripServo.setPosition(0.9);
                                })
                                .splineToLinearHeading(new Pose2d(36, -14, Math.toRadians(90)), Math.toRadians(90))
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