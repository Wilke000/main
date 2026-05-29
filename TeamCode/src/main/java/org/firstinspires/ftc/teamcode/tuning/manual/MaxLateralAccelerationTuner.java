package org.firstinspires.ftc.teamcode.tuning.manual;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import drivetrains.Drivetrain;
import localizers.Localizer;
import geometry.Pose;

/**
 * Empirical OpMode to determine the maxLateralAccel constant for centripetal path clamping.
 * <p>
 * How to use:
 * 1. Place the robot in an open space on the foam tiles.
 * 2. Push Left Stick FORWARD and Right Stick LEFT/RIGHT to make the robot drive in a steady circle.
 * 3. Slowly increase your speed.
 * 4. Watch the robot closely. The moment the wheels lose traction and the back end "washes out" (drifts sideways),
 * look at the peak "Max Lateral Acceleration Achieved" value on your telemetry.
 * 5. That peak value is your physical limit. Input it into your BSplineFollowerConstants.
 * @author Sohum Arora - 22985 Paraducks
 */
@Configurable
@TeleOp(name = "Max Lateral Acceleration Tuner", group = "Apex Pathing Tuning")
public class MaxLateralAccelerationTuner extends OpMode {

    // Injecting dependencies matching your codebase style
    private Drivetrain drivetrain;
    private Localizer localizer;
    private JoinedTelemetry fullTelem;

    // Tracking variables for kinematics calculations
    private long lastTimeNs = 0;
    private Pose lastPose = null;

    // Low-pass filter to smooth out sensor noise on velocity calculations
    private double filteredAccel = 0.0;
    private final double FILTER_GAIN = 0.2; // Adjust between 0.0 and 1.0 (lower means smoother)

    // Peak hold tracker
    private double maxAccelAchieved = 0.0;

    @Override
    public void init() {
        // Replace these with your team's hardware map initialization methods
        Constants constants = new Constants();
        drivetrain = constants.buildOnlyDrivetrain(hardwareMap);
        localizer = constants.buildOnlyLocalizer(hardwareMap, Pose.zero());
        fullTelem = new JoinedTelemetry(telemetry);

        lastTimeNs = System.nanoTime();
        lastPose = localizer.getPose();

        fullTelem.addLine("Tuner Initialized. Place robot in an open area on the mats.");
        fullTelem.update();
    }

    @Override
    public void init_loop() {
        localizer.update();
    }

    @Override
    public void loop() {
        // 1. Update odometry position tracking
        localizer.update();
        long currentTimeNs = System.nanoTime();
        Pose currentPose = localizer.getPose();

        // 2. Compute delta time in seconds
        double dt = (currentTimeNs - lastTimeNs) / 1e9;

        if (dt > 1e-6 && lastPose != null) {
            // Calculate linear displacement delta (inches)
            double dx = currentPose.getX().getIn() - lastPose.getX().getIn();
            double dy = currentPose.getY().getIn() - lastPose.getY().getIn();
            double linearDistance = Math.hypot(dx, dy);

            // Calculate angular displacement delta (radians)
            double dTheta = currentPose.getHeading().getRad() - lastPose.getHeading().getRad();

            // Normalize angular delta to handle wrap-around angles [-PI, PI]
            dTheta = (dTheta + Math.PI) % (2 * Math.PI) - Math.PI;

            // 3. Convert displacements to real-time physical velocities
            double linearVelocity = linearDistance / dt;  // in/s
            double angularVelocity = dTheta / dt;        // rad/s

            // 4. Calculate instantaneous lateral (centripetal) acceleration: a_c = v * w
            double rawAccel = Math.abs(linearVelocity * angularVelocity); // in/s^2

            // Apply a low-pass filter to prevent telemetry flicker
            filteredAccel = (FILTER_GAIN * rawAccel) + ((1.0 - FILTER_GAIN) * filteredAccel);

            // 5. Track peak hold (only update if the robot is actually driving hard)
            if (filteredAccel > maxAccelAchieved && linearVelocity > 5.0) {
                maxAccelAchieved = filteredAccel;
            }
        }

        // Save current states for the next loop cycle
        lastTimeNs = currentTimeNs;
        lastPose = currentPose;

        // 6. Manual Driving Control (Standard Arcade/Mecanum input)
        // Drive forward/backward with left stick, rotate with right stick
        double forwardPower = -gamepad1.left_stick_y;
        double turnPower = -gamepad1.right_stick_x;

        // Feed raw stick inputs directly to drivetrain
        // (Assuming 0.0 for strafe power so we get a uniform traction profile)
        drivetrain.drive(forwardPower, 0.0, turnPower, currentPose.getHeading().getRad());

        // 7. Reset peak hold tracker if driver presses 'A'
        if (gamepad1.a) {
            maxAccelAchieved = 0.0;
        }

        // 8. Telemetry output
        fullTelem.addData("Live Lateral Accel (in/s^2)", Math.round(filteredAccel * 10.0) / 10.0);
        fullTelem.addData("★ MAX LATERAL ACCELERATION ACHIEVED ★", Math.round(maxAccelAchieved * 10.0) / 10.0);
        fullTelem.addLine("\n--- Diagnostics ---");
        fullTelem.addData("Linear Speed (in/s)", Math.round(Math.hypot(forwardPower, turnPower) * 100.0) / 100.0);
        fullTelem.addLine("Press 'A' on Gamepad 1 to reset peak tracker.");
        fullTelem.update();
    }

    @Override
    public void stop() {
        drivetrain.stop();
    }
}