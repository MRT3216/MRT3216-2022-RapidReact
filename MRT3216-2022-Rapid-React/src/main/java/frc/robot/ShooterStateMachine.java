package frc.robot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.Ball;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import io.github.oblarg.oblog.Loggable;

public class ShooterStateMachine implements Loggable {
    private static ShooterStateMachine instance;
    private Ball ball1st;
    private Ball ball2nd;
    private ShooterSubsystem shooterSystem;
    private ColorSensorSubsystem colorSystem;
    private double ballShotFilterThreshold;
    private double ballShotDebounceTime;

    private ShooterStateMachine() {
        this.shooterSystem = ShooterSubsystem.getInstance();
        this.colorSystem = ColorSensorSubsystem.getInstance();
        this.ballShotFilterThreshold = Constants.Shooter.Flywheel.ballShotfilterThreshold;
        this.ballShotDebounceTime = Constants.Shooter.Flywheel.ballShotDebounceTime;
        ball1st = Ball.NONE;
        ball2nd = Ball.NONE;

        new Trigger(
                () -> shooterSystem.getFilterValue() < this.ballShotFilterThreshold)
                        .debounce(this.ballShotDebounceTime, Debouncer.DebounceType.kRising)
                        .whileActiveOnce(new InstantCommand(() -> ballShot()));

        new Trigger(() -> colorSystem.inRange())
                .whileActiveOnce(new InstantCommand(() -> ballIndexed(colorSystem.isAllianceBall())));
    }

    public void ballShot() {
        ball1st = ball2nd;
        ball2nd = Ball.NONE;
    }

    public void ballIndexed(boolean allianceBall) {
        ball1st = allianceBall ? Ball.ALLIANCE : Ball.OPPONENT;
    }

    public void reverseEject() {
        ball1st = Ball.NONE;
        ball2nd = Ball.NONE;
    }

    public Ball get1stBall() {
        return ball1st;
    }

    public String get1stBallString() {
        return ball1st.toString();
    }

    public Ball get2ndBall() {
        return ball2nd;
    }

    public String get2ndBallString() {
        return ball2nd.toString();
    }

    public void set1stBall(Ball ball) {
        this.ball1st = ball;
    }

    public void set2ndBall(Ball ball) {
        this.ball2nd = ball;
    }

    public void setBallFilterThreshold(double ballShotFilterThreshold) {
        this.ballShotFilterThreshold = ballShotFilterThreshold;
    }

    public void setBallShotDebounceTime(double ballShotDebounceTime) {
        this.ballShotDebounceTime = ballShotDebounceTime;
    }

    public static ShooterStateMachine getInstance() {
        if (instance == null) {
            // if instance is null, initialize
            instance = new ShooterStateMachine();
        }
        return instance;
    }
}