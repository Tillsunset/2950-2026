package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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

    public static double rpmFromInches(double inches) {
        double offsetToCenter = inches + 36;
        double rpmGuess = offsetToCenter * 14.88 + 1425;
        return MathUtil.clamp(rpmGuess, 1500, 3500);
    }

    public static double rpmFromMeters(double meters) {
        return rpmFromInches(meters * 39.37);
    }
}
