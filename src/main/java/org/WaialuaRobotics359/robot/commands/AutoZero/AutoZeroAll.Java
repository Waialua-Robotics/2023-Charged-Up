public class AutoZeroAll extends CommandBase {
    private Wrist s_Wrist;
    private Elevator s_Elevator;
    private Slide s_Slide;

    private static int ElevatorPosition;
    private static int SlidePosition;
    private static int WristPosition;

    public AutoZeroAll(Wrist s_Wrist, Elevator s_Elevator, Slide s_Slide) {
        this.s_Wrist = s_Wrist;
        this.s_Elevator = s_Elevator;
        this.s_Slide = s_Slide;
        addRequirements(s_Wrist);
        addRequirements(s_Elevator);
        addRequirements(s_Slide);
    }

    boolean finished = false; 

    private Timer Timer = new Timer();

    public void initialize(){
        
        finished = false;
        Timer.reset();
        Timer.start();

        AutoZeroWrist(s_Wrist);
        AutoZeroElevator(s_Elevator);
        AutoZeroSlide(s_Slide);

    }

    @Override
    public void execute(){

        if (s_Wrist.AutoZeroWrist.isFinished() && s_Elevator.isFinished() && s_Slide.isFinished()){
            finished = true;
        }
        
        if (Timer.hasElapsed(5)){
            finished = true;
            reportWarning("AutoZeroAll timed out", false);
        }
    }
    
    public boolean isFinished(){
        return finished;
    }
}