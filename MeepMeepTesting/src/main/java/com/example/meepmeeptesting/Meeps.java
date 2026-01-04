package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Meeps {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity redFar = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .build();

        RoadRunnerBotEntity blueFar = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .build();

        RoadRunnerBotEntity redClose = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .build();

        RoadRunnerBotEntity blueClose = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .build();

        redFar.runAction(redFar.getDrive().actionBuilder(new Pose2d(62, -15, Math.PI))
                .lineToX(55)
                .turnTo(3.6)
                .turnTo(Math.PI)
                .lineToX(40)
                .build());

        blueFar.runAction(blueFar.getDrive().actionBuilder(new Pose2d(62, 15, Math.PI))
                .lineToX(55)
                .turnTo(2.85)
                .setTangent(Math.PI)
                .splineToLinearHeading(new Pose2d(36, 30, Math.PI/2), Math.PI/2)
                .lineToY(50)
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(55, 15, 2.85), 2.85-Math.PI)
                //.turnTo(Math.PI/2)
                //.splineToConstantHeading(new Vector2d(61, 60), Math.PI/2)
                //.setTangent(Math.PI/2)
                //.splineToLinearHeading(new Pose2d(61, 61, Math.PI/2), Math.PI/2)
                //.setTangent((Math.PI*2)-1.85)
                //splineToLinearHeading(new Pose2d(55, 15, 2.85), (Math.PI/2)*3)
                .build());

        redClose.runAction(redClose.getDrive().actionBuilder(new Pose2d(-51, 49.5, Math.toRadians(307)))
                .lineToX(-48)
                .waitSeconds(1)
                .turnTo(0)
                .waitSeconds(1)
                .lineToX(-30)
                .build());
/*
        blueClose.runAction(blueClose.getDrive().actionBuilder(new Pose2d(-51, -49.5, Math.toRadians(53)))
                .lineToX(-48)
                .waitSeconds(1)
                .turnTo(0)
                .waitSeconds(1)
                .lineToX(-30)
                .build());
*/
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                //.addEntity(redFar)
                .addEntity(blueFar)
                //.addEntity(redClose)
                //.addEntity(blueClose)
                .start();
    }
}