package org.firstinspires.ftc.teamcode.tuning.manual;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;

import drivetrains.Drivetrain;
import followers.BSplineFollower;
import localizers.Localizer;
import paths.Path;
import paths.PathBuilder;
import util.Pose;

/**
 * OpMode for testing the BSpline follower based on values in Constants.java.
 * Hold X to execute the multi-stage B-Spline test path forward,
 * and hold A to reset and drive back to the start position.
 *
 * @author Sohum Arora - 22985 Paraducks
 */
@TeleOp(name = "BSpline Tester", group = "Apex Pathing Tuning")
public class BSplineTuner extends OpMode {
    private Drivetrain drivetrain;
    private Localizer localizer;
    private BSplineFollower follower;
    private JoinedTelemetry fullTelem;

    private Path currentPath;
    private boolean pathActive = false;
    private boolean wasAtTarget = false;

    @Override
    public void init() {
        // Build constants, drivetrain, localizer, and telemetry
        Constants constants = new Constants();
        drivetrain = constants.buildOnlyDrivetrain(hardwareMap);
        localizer = constants.buildOnlyLocalizer(hardwareMap, Pose.zero());
        fullTelem = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        // Build the follower using the pre-configured constants natively from Constants.java
        follower = (BSplineFollower) constants.build(hardwareMap, Pose.zero());

        fullTelem.addLine(
                "Hold X to run the multi-stage B-Spline test path, and A to drive back to the start position."
        );
        fullTelem.update();
    }

    private void runPath(boolean forward) {
        if (!pathActive) {
            if (forward) {
                currentPath = new PathBuilder(localizer.getPose())
                        .holdPose(1.5)
                        .build();
            } else {
                currentPath = new PathBuilder(localizer.getPose())
                        .build();
            }
            follower.followPath(currentPath);
            pathActive = true;
        }
        follower.update();
    }

    @Override
    public void loop() {
        localizer.update();

        if (gamepad1.x) { // Run the test path forward when X is held
            runPath(true);
        } else if (gamepad1.a) { // Move back to start position when A is held
            runPath(false);
        } else {
            follower.stop();
            drivetrain.stop();
            pathActive = false;
        }

        boolean atTarget = pathActive && !follower.isBusy();
        if (atTarget && !wasAtTarget) { // Gamepad rumble and Led green when at target
            gamepad1.rumble(0.5, 0.5, 100);
            gamepad1.setLedColor(0, 1, 0, 300);
            pathActive = false; // Release active state to allow re-triggering
        } else if (pathActive && !atTarget) { // Led red when not at target
            gamepad1.setLedColor(1, 0, 0, 100);
        }
        wasAtTarget = atTarget;

        fullTelem.addData("Target Path: ", pathActive ? "Active" : "Inactive");
        fullTelem.addData("Position X: ", localizer.getPose().getX());
        fullTelem.addData("Position Y: ", localizer.getPose().getY());
        fullTelem.addData("Heading: ", localizer.getPose().getHeading());
        fullTelem.addData("At Target: ", atTarget);
        fullTelem.addData("Drivetrain Output: ", drivetrain.toString());
        fullTelem.update();
    }
}