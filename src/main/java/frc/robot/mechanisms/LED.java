package frc.robot.mechanisms;

import edu.wpi.first.wpilibj.PWM;


public class LED {
    private int channel = 0;
    private PWM pwm = null;
    private LEDStatus ledStatus = LEDStatus.ready;

    public static enum LEDStatus {
        ready,
        problem,
        targetAquired,
        targetSearching,
        hasCoral,
        hasAlgae
    }

    public LED(int channel) {
        this.channel = channel;

        pwm = new PWM(channel);
        setStatus(LEDStatus.ready);
    }

    public void setStatus(LEDStatus ledStatus) {
        this.ledStatus = ledStatus;

        switch(ledStatus) {
            case ready:
                //System.out.println("setting led to ready");
                pwm.setPulseTimeMicroseconds(1855);
                break;
            case problem:
                //System.out.println("setting led to problem");
                pwm.setPulseTimeMicroseconds(1795);
                break;
            case targetAquired:
                //System.out.println("setting led to targetAquired");
                pwm.setPulseTimeMicroseconds(1475);
                break;
            case targetSearching:
                //System.out.println("setting led to targetSearching");
                pwm.setPulseTimeMicroseconds(1345);
                break;
            case hasCoral:
                pwm.setPulseTimeMicroseconds(1475);
                break;
            case hasAlgae:
                pwm.setPulseTimeMicroseconds(1475);
                break;
            default:
                break;
        }
    }
}
