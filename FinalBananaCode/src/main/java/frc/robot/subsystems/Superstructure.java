package frc.robot.subsystems;

public class Superstructure {
    private static Superstructure instance;

    public enum RobotState {
        DRIVE,
        INTAKE,
        SPEAKER,
        AMP,
        CHAIN
    }

    private RobotState currentState;

    private final Shooter shooter = Shooter.getInstance();
    private final Drive drive = Drive.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Arm arm = Arm.getInstance();

    private Superstructure() {
        currentState = RobotState.DRIVE; 
    }

    public static Superstructure getInstance() {
        if (instance == null) {
            instance = new Superstructure();
        }
        return instance;
    }

    public void setState(RobotState newState) {
        currentState = newState;
        switch (newState) {
            case DRIVE:
                break;
            case INTAKE:
                break;
        }
    }

    public RobotState getState() {
        return currentState;
    }
}
