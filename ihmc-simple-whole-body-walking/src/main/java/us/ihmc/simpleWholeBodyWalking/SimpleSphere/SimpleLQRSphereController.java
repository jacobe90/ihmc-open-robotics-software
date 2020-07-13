package us.ihmc.simpleWholeBodyWalking.SimpleSphere;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.commonWalkingControlModules.capturePoint.lqrControl.LQRMomentumController;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryProvider;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationConstructionSetTools.tools.RobotTools;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.simpleWholeBodyWalking.SimpleBipedCoMTrajectoryPlanner;

public class SimpleLQRSphereController implements SimpleSphereControllerInterface
{
   private final YoVariableRegistry registry = new YoVariableRegistry("SphereController");

   private final RobotTools.SCSRobotFromInverseDynamicsRobotModel scsRobot;
   private final SimpleSphereRobot sphereRobot;
   private final ExternalForcePoint externalForcePoint;

   private final LQRMomentumController lqrMomentumController;

   private final YoFrameVector3D lqrForce = new YoFrameVector3D("lqrForce", ReferenceFrame.getWorldFrame(), registry);

   private final SimpleBipedCoMTrajectoryPlanner dcmPlan;
   private final List<ContactStateProvider> contactStateProviders = new ArrayList<>();

   public SimpleLQRSphereController(SimpleSphereRobot sphereRobot, SimpleBipedCoMTrajectoryPlanner comTrajectoryProvider, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.scsRobot = sphereRobot.getScsRobot();
      this.sphereRobot = sphereRobot;
      externalForcePoint = sphereRobot.getScsRobot().getAllExternalForcePoints().get(0);

      dcmPlan = comTrajectoryProvider;

      sphereRobot.getScsRobot().setController(this);

      lqrMomentumController = new LQRMomentumController(sphereRobot.getOmega0Provider(), registry);
   }

   private final DMatrixRMaj currentState = new DMatrixRMaj(6, 1);

   @Override
   public void doControl()
   {
      scsRobot.updateJointPositions_SCS_to_ID();
      scsRobot.updateJointVelocities_SCS_to_ID();

      sphereRobot.updateFrames();

      double currentTime = sphereRobot.getScsRobot().getYoTime().getDoubleValue();
      dcmPlan.compute(currentTime, sphereRobot.getCenterOfMass());

      sphereRobot.getDesiredDCM().set(dcmPlan.getDesiredDCMPosition());
      sphereRobot.getDesiredDCMVelocity().set(dcmPlan.getDesiredDCMVelocity());

      lqrMomentumController.setVRPTrajectory(dcmPlan.getVRPTrajectories());
      sphereRobot.getCenterOfMass().get(currentState);
      sphereRobot.getCenterOfMassVelocity().get(3, currentState);
      lqrMomentumController.computeControlInput(currentState, currentTime);

      lqrForce.set(lqrMomentumController.getU());
      lqrForce.addZ(sphereRobot.getGravityZ());
      lqrForce.scale(sphereRobot.getTotalMass());

      externalForcePoint.setForce(lqrForce);

      scsRobot.updateJointPositions_ID_to_SCS();
      scsRobot.updateJointVelocities_ID_to_SCS();
      scsRobot.updateJointTorques_ID_to_SCS();
   }

   public void solveForTrajectory()
   {
      dcmPlan.initialize();
      dcmPlan.solveForTrajectory(sphereRobot.getScsRobot().getYoTime().getDoubleValue(), sphereRobot.getCenterOfMass());
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getDescription()
   {
      return registry.getName();
   }

   @Override
   public String getName()
   {
      return registry.getName();
   }

}