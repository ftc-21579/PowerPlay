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
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.73622)
                .setDimensions(13, 18)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(90)))
                                .addDisplacementMarker(() -> {

                                    // Lift up
                                    // Lift slightly down to get closer
                                    // Release claw
                                })
                                .lineToLinearHeading(new Pose2d(-35, -10, Math.toRadians(45))) // Move to align with pole
                                .waitSeconds(1.5) // Simulate lift
                                .back(2.5) // Back away from pole/align with stack
                                .addDisplacementMarker(() -> {
                                    // Lift all the way down to height for cone
                                })
                                .turn(Math.toRadians(135)) // Turn to face cone stack
                                //.setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, Math.toRadians(308.9955575516127), 12.73622)) // 15in/s
                                .resetVelConstraint()
                                .forward(20) // Get closer to stack
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(5, Math.toRadians(308.9955575516127), 12.73622)) // 5in/s
                                .forward(4) // get right on stack
                                .addDisplacementMarker(() -> {
                                    // Close claw
                                    // Lift up a little bit
                                })
                                .waitSeconds(1) // Simulate lift
                                .back(4) // Slowly away from stack
                                //.setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, Math.toRadians(308.9955575516127), 12.73622)) // 15in/s
                                .resetVelConstraint()
                                .addDisplacementMarker(() -> {
                                    // Lift all the way up
                                })
                                //.waitSeconds(2.5) // Simulate lift
                                .back(22) // Back towards pole
                                .resetVelConstraint() // Back to 60in/s
                                .turn(Math.toRadians(-135)) // Turn to face pole
                                .forward(2.5) // Get closer to pole
                                .addDisplacementMarker(() -> {
                                    // Lift down a little
                                    // Release cone
                                })
                                .waitSeconds(1) // Simulate lift
                                .back(2.5) // Back away from pole/align with stack
                                .addDisplacementMarker(() -> {
                                    // Lift all the way down
                                })
                                .turn(Math.toRadians(135)) // Turn to face cone stack
                                //.setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, Math.toRadians(308.9955575516127), 12.73622)) // 15in/s
                                .resetVelConstraint()
                                .forward(22) // Get closer to stack
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(5, Math.toRadians(308.9955575516127), 12.73622)) // 5in/s
                                .forward(4) // get right on stack
                                .addDisplacementMarker(() -> {
                                    // Close claw
                                    // Lift up a little bit
                                })
                                .waitSeconds(1) // Simulate lift
                                .back(4) // Slowly away from stack
                                //.setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, Math.toRadians(308.9955575516127), 12.73622)) // 15in/s
                                .resetVelConstraint()
                                .addDisplacementMarker(() -> {
                                    // Lift all the way up
                                })
                                .back(22) // Back towards pole
                                .turn(Math.toRadians(-135)) // Turn to face pole
                                .forward(2.5) // Get closer to pole
                                .addDisplacementMarker(() -> {
                                    // Lift down a little
                                    // Release cone
                                })
                                .waitSeconds(1.5) // Simulate lift
                                .back(2.5)
                                .addDisplacementMarker(() -> {
                                    // Lift all the way down
                                })
                                .resetVelConstraint() // 60in/s
                                .turn(Math.toRadians(45))
                                .strafeLeft(26)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}