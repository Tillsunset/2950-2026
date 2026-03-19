package frc.robot;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import limelight.Limelight;
import limelight.results.RawFiducial;

public class Helper {

    public static boolean isOurHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            // If we have no alliance, we cannot be enabled, therefore no hub.
            return false;
        } else if (DriverStation.isAutonomousEnabled()) {
            // Hub is always enabled in autonomous.
            return true;
        } else if (!DriverStation.isTeleopEnabled()) {
            // At this point, if we're not teleop enabled, there is no hub.
            return false;
        }

        // We're teleop enabled, compute.
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its
        // likely early in teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                // If we have invalid game data, assume hub is active.
                return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        double matchTime = DriverStation.getMatchTime();

        if (matchTime > 130) {
            // Transition shift, hub is active.
            return true;
        } else if (matchTime > 105) {
            // Shift 1
            return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
        }
    }

    public static double timeTillShift() {
        double matchTime = DriverStation.getMatchTime();

        return (matchTime - 30) % 25;
    }

    public static double rpmFromMeters(double meters) {
        double x1 = 1.0;
        double y1 = 2500.0;
        double x2 = 1.5;
        double y2 = 3000.0;
        double x3 = 2.0;
        double y3 = 3500.0;

        double rpmGuess = 2400.0;

        if (meters > x2) {
            rpmGuess = y2 + (meters - x2) * (y3 - y2) / (x3-x2);
            return MathUtil.clamp(rpmGuess, 2400, 3500);
        }
        else { // meters < x2
            rpmGuess = y1 + (meters - x1) * (y2 - y1) / (x2-x1);
            return MathUtil.clamp(rpmGuess, 2400, 3500);
        }
    }

	private static int i = 0;

    public static void printRPMDistance(double rpm, double distance) {
        if (i % 10 == 0) {
			System.out.println(rpm);
			System.out.println(distance);
			System.out.println("*****************");
		}

		++i;
    }

    private static LinearFilter distFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    private static LinearFilter aimFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

	private static Limelight ll = new Limelight("limelight");

	private static double xOffset = 0;

    public static void LLSetup() {
        ll.getSettings()
		.withAprilTagIdFilter(List.of(2, 5, 10, 18, 21, 26))
		.save();
    }

    public static void updateFilters() {
        RawFiducial[] raw = ll.getData().getRawFiducials();
        for (RawFiducial object : raw){
            distFilter.calculate(object.distToCamera);
            aimFilter.calculate(object.txnc);
        }
    }
    
    public static double getAprilTagDist() {
        return distFilter.lastValue();
    }
    
    public static double getAprilTagAim() {
        return aimFilter.lastValue() - xOffset;
    }

}
