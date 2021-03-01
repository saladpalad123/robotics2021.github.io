package frc.robot.utils;
import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj.controller.PIDController;
import static frc.robot.Constants.*;

public class Vision {
    public final PhotonCamera camera = new PhotonCamera(AutoAimConstants.CAMERA_NAME);
    public final PIDController controller = new PIDController(AutoAimConstants.KP, AutoAimConstants.KI, AutoAimConstants.KD);
}