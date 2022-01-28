package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // Declare our first bot
        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0, 48, 0))
                               // .forward(30)
                               // .turn(Math.toRadians(90))
                              //  .forward(30)
                              //  .turn(Math.toRadians(90))
                              //  .forward(30)
                              //  .turn(Math.toRadians(90))
                             //   .forward(30)
                             //   .turn(Math.toRadians(90))
                                 .splineTo(new Vector2d(-48, 48), Math.toRadians(0))
                                 .splineTo(new Vector2d(-10, 48), Math.toRadians(0))
                                .splineTo(new Vector2d(-10, 36), Math.toRadians(90))
                               // .splineTo(new Vector2d(-10, 48), Math.toRadians(-90))
                                .splineTo(new Vector2d(-10, 65), Math.toRadians(-90))
                                .strafeLeft(48)
                                .strafeRight(48)
                                .strafeLeft(48)
                                .strafeRight(48)
                                .strafeLeft(56)


                                // add a spline for cycling
                               // .splineTo(new Vector2d(-10, 65), Math.toRadians(-90))
                                .build()
                );

        // Declare out second bot
        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(30, 30, Math.toRadians(180)))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                // Add both of our declared bot entities
                .addEntity(myFirstBot)
              //  .addEntity(mySecondBot)
                .start();
    }
}