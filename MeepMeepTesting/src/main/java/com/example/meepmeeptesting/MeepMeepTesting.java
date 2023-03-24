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
        double grabTime = 0.25;
        double stackTime1 = 0.25;
        double stackTime2 = 0.25;
        double scoreTime1 = 0.25;
        double scoreTime2 = 0.25;
        double scoreTime3 = 0.25;
        double startx = -36;
        double starty = -64;
        double scorex = -32;
        double scorey = -8;
        double stackx = -60;
        double stacky = -12;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(160), Math.toRadians(160), 13.95)
                .setDimensions(13, 18)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(startx, starty, Math.toRadians(90)))
                                .addTemporalMarker(0.75, () -> {
                                    //signal.setPosition(0.0);
                                })
                                .addTemporalMarker(() -> {
                                    //targetInches = 0;
                                    //signal.setPosition(0.5);
                                    //gripServo.setPosition(closed);
                                })
                                .waitSeconds(grabTime)
                                .addTemporalMarker(() -> {
                                    //targetInches = high;
                                })
                                .splineTo(new Vector2d(-36, -24), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(scorex, scorey, Math.toRadians(45)), Math.toRadians(45))
                                .waitSeconds(scoreTime1)
                                .addTemporalMarker(() -> {
                                    //guide.setPosition(extended);
                                })
                                .waitSeconds(scoreTime2)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(open);
                                    //guide.setPosition(retracted);
                                    //signal.setPosition(0.5);
                                    //targetInches = stack1;
                                })
                                .waitSeconds(scoreTime3)
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-48, stacky, Math.toRadians(180)), Math.toRadians(180))
                                .splineTo(new Vector2d(stackx, stacky), Math.toRadians(180))
                                .waitSeconds(stackTime1)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(closed);
                                })
                                .waitSeconds(stackTime2)
                                .addTemporalMarker(() -> {
                                    //targetInches = high;
                                })
                                .splineTo(new Vector2d(-48, stacky), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(scorex, scorey, Math.toRadians(45)), Math.toRadians(45))
                                .waitSeconds(scoreTime1)
                                .addTemporalMarker(() -> {
                                    //guide.setPosition(extended);
                                })
                                .waitSeconds(scoreTime2)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(open);
                                    //guide.setPosition(retracted);
                                    //targetInches = stack2;
                                })
                                .waitSeconds(scoreTime3)
                                .splineToSplineHeading(new Pose2d(-48, stacky, Math.toRadians(180)), Math.toRadians(180))
                                .splineTo(new Vector2d(stackx - 0.125, stacky), Math.toRadians(180))
                                .waitSeconds(stackTime1)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(closed);
                                })
                                .waitSeconds(stackTime2)
                                .addTemporalMarker(() -> {
                                    //targetInches = high;
                                })
                                .splineTo(new Vector2d(-48, stacky), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(scorex, scorey, Math.toRadians(45)), Math.toRadians(45))
                                .waitSeconds(scoreTime1)
                                .addTemporalMarker(() -> {
                                    //guide.setPosition(extended);
                                })
                                .waitSeconds(scoreTime2)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(open);
                                    //guide.setPosition(retracted);
                                    //targetInches = stack3;
                                })
                                .waitSeconds(scoreTime3)
                                .splineToSplineHeading(new Pose2d(-48, stacky, Math.toRadians(180)), Math.toRadians(180))
                                .splineTo(new Vector2d(stackx - 0.25, stacky), Math.toRadians(180))
                                .waitSeconds(stackTime1)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(closed);
                                })
                                .waitSeconds(stackTime2)
                                .addTemporalMarker(() -> {
                                    //targetInches = high;
                                })
                                .splineTo(new Vector2d(-48, stacky), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(scorex, scorey, Math.toRadians(45)), Math.toRadians(45))
                                .waitSeconds(scoreTime1)
                                .addTemporalMarker(() -> {
                                    //guide.setPosition(extended);
                                })
                                .waitSeconds(scoreTime2)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(0.9);
                                    //guide.setPosition(retracted);
                                    //targetInches = 0;
                                })
                                .waitSeconds(scoreTime3)
                                .splineTo(new Vector2d(-36, -24), Math.toRadians(270))
                                .splineToConstantHeading(new Vector2d(-48, -36), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-60, -36), Math.toRadians(180))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}