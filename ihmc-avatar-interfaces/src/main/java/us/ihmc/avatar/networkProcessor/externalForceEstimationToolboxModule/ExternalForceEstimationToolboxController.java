package us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule;

import static us.ihmc.robotModels.FullRobotModelUtils.getAllJointsExcludingHands;

import controller_msgs.msg.dds.*;
import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.map.hash.TIntObjectHashMap;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule.detector.ContactParticleFilter;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBodyTools;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.DynamicsMatrixCalculator;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.externalForceEstimationToolboxAPI.ExternalForceEstimationToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialVector;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.ToIntFunction;

public class ExternalForceEstimationToolboxController extends ToolboxController
{
   private final YoBoolean estimateContactPosition = new YoBoolean("estimateContactPosition", registry);
   private final HumanoidReferenceFrames referenceFrames;

   private final AtomicReference<RobotConfigurationData> robotConfigurationData = new AtomicReference<>();
   private final AtomicReference<RobotDesiredConfigurationData> robotDesiredConfigurationData = new AtomicReference<>();

   private final FullHumanoidRobotModel fullRobotModel;
   private final FloatingJointBasics rootJoint;
   private final OneDoFJointBasics[] oneDoFJoints;

   private final TIntObjectHashMap<RigidBodyBasics> rigidBodyHashMap = new TIntObjectHashMap<>();
   private final HashMap<String, OneDoFJointBasics> jointNameMap = new HashMap<>();
   private final ToIntFunction<String> jointNameToMatrixIndexFunction;
   private final YoBoolean calculateRootJointWrench = new YoBoolean("calculateRootJointWrench", registry);

   private final DynamicsMatrixCalculator dynamicsMatrixCalculator;

   private final DMatrixRMaj controllerDesiredQdd;
   private final DMatrixRMaj controllerDesiredTau;

   private final DMatrixRMaj massMatrix;
   private final DMatrixRMaj coriolisGravityMatrix;

   private final CommandInputManager commandInputManager;
   private final ExternalForceEstimationOutputStatus outputStatus = new ExternalForceEstimationOutputStatus();

   private PredefinedContactExternalForceSolver predefinedContactForceSolver;
   private ContactParticleFilter contactParticleFilter;

   public ExternalForceEstimationToolboxController(DRCRobotModel robotModel,
                                                   FullHumanoidRobotModel fullRobotModel,
                                                   CommandInputManager commandInputManager,
                                                   StatusMessageOutputManager statusOutputManager,
                                                   YoGraphicsListRegistry graphicsListRegistry,
                                                   int updateRateMillis,
                                                   YoRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);

      this.commandInputManager = commandInputManager;
      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      this.oneDoFJoints = getAllJointsExcludingHands(fullRobotModel);
      this.rootJoint = fullRobotModel.getRootJoint();

      MultiBodySystemTools.getRootBody(fullRobotModel.getElevator())
                          .subtreeIterable()
                          .forEach(rigidBody -> rigidBodyHashMap.put(rigidBody.hashCode(), rigidBody));

      double updateDT = Conversions.millisecondsToSeconds(updateRateMillis);
      JointBasics[] joints = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullRobotModel);
      WholeBodyControlCoreToolbox controlCoreToolbox = new WholeBodyControlCoreToolbox(robotModel.getControllerDT(),
                                                                9.81,
                                                                fullRobotModel.getRootJoint(),
                                                                joints,
                                                                referenceFrames.getCenterOfMassFrame(),
                                                                robotModel.getWalkingControllerParameters().getMomentumOptimizationSettings(),
                                                                graphicsListRegistry,
                                                                parentRegistry);

      ArrayList<ContactablePlaneBody> contactablePlaneBodies = new ArrayList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics footBody = fullRobotModel.getFoot(robotSide);
         ReferenceFrame soleFrame = fullRobotModel.getSoleFrame(robotSide);
         contactablePlaneBodies.add(ContactablePlaneBodyTools.createTypicalContactablePlaneBodyForTests(footBody, soleFrame));
      }
      controlCoreToolbox.setupForInverseDynamicsSolver(contactablePlaneBodies);

      this.dynamicsMatrixCalculator = new DynamicsMatrixCalculator(controlCoreToolbox);
      int degreesOfFreedom = Arrays.stream(joints).mapToInt(JointReadOnly::getDegreesOfFreedom).sum();

      this.controllerDesiredQdd = new DMatrixRMaj(degreesOfFreedom, 1);
      this.controllerDesiredTau = new DMatrixRMaj(degreesOfFreedom, 1);

      this.massMatrix = new DMatrixRMaj(degreesOfFreedom, degreesOfFreedom);
      this.coriolisGravityMatrix = new DMatrixRMaj(degreesOfFreedom, 1);

      ForceEstimatorDynamicMatrixUpdater dynamicMatrixUpdater = (massMatrix, coriolisGravityMatrix, tau) ->
      {
         massMatrix.set(this.massMatrix);
         coriolisGravityMatrix.set(this.coriolisGravityMatrix);
         tau.set(controllerDesiredTau);
      };

      RobotCollisionModel collisionModel = robotModel.getHumanoidRobotKinematicsCollisionModel();
      List<Collidable> collidables = collisionModel.getRobotCollidables(fullRobotModel.getRootBody());

      predefinedContactForceSolver = new PredefinedContactExternalForceSolver(joints, updateDT, dynamicMatrixUpdater, graphicsListRegistry, registry);
      contactParticleFilter = new ContactParticleFilter(joints, updateDT, dynamicMatrixUpdater, collidables, graphicsListRegistry, registry);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         jointNameMap.put(oneDoFJoints[i].getName(), oneDoFJoints[i]);
      }

      jointNameToMatrixIndexFunction = jointName ->
      {
         OneDoFJointBasics joint = jointNameMap.get(jointName);
         return dynamicsMatrixCalculator.getMassMatrixCalculator().getInput().getJointMatrixIndexProvider().getJointDoFIndices(joint)[0];
      };
   }

   @Override
   public boolean initialize()
   {
      if (commandInputManager.isNewCommandAvailable(ExternalForceEstimationToolboxConfigurationCommand.class))
      {
         ExternalForceEstimationToolboxConfigurationCommand configurationCommand = commandInputManager.pollNewestCommand(
               ExternalForceEstimationToolboxConfigurationCommand.class);

         estimateContactPosition.set(configurationCommand.getEstimateContactLocation());

         if (estimateContactPosition.getBooleanValue())
         {
            RigidBodyBasics rigidBody = rigidBodyHashMap.get(configurationCommand.getRigidBodyHashCodes().get(0));
            contactParticleFilter.setLinkToEstimate(rigidBody);
            contactParticleFilter.initialize();
         }
         else
         {
            predefinedContactForceSolver.clearContactPoints();
            int numberOfContactPoints = configurationCommand.getNumberOfContactPoints();
            for (int i = 0; i < numberOfContactPoints; i++)
            {
               RigidBodyBasics rigidBody = rigidBodyHashMap.get(configurationCommand.getRigidBodyHashCodes().get(i));
               Point3D contactPoint = configurationCommand.getContactPointPositions().get(i);
               predefinedContactForceSolver.addContactPoint(rigidBody, contactPoint, true);
            }

            calculateRootJointWrench.set(configurationCommand.getCalculateRootJointWrench());
            if(calculateRootJointWrench.getValue())
            {
               predefinedContactForceSolver.addContactPoint(fullRobotModel.getRootBody(), new Vector3D(), false);
            }

            predefinedContactForceSolver.setEstimatorGain(configurationCommand.getEstimatorGain());
            predefinedContactForceSolver.setSolverAlpha(configurationCommand.getSolverAlpha());
         }

         predefinedContactForceSolver.initialize();

         commandInputManager.clearCommands(ExternalForceEstimationToolboxConfigurationCommand.class);
      }
      else if (predefinedContactForceSolver == null)
      {
         return false;
      }

      return true;
   }

   @Override
   public void updateInternal()
   {
      RobotConfigurationData robotConfigurationData = this.robotConfigurationData.getAndSet(null);
      if (robotConfigurationData != null)
      {
         updateRobotState(robotConfigurationData, rootJoint, oneDoFJoints);
         referenceFrames.updateFrames();
      }

      RobotDesiredConfigurationData desiredConfigurationData = this.robotDesiredConfigurationData.getAndSet(null);
      if (desiredConfigurationData != null)
      {
         updateRobotDesiredState(desiredConfigurationData, controllerDesiredQdd, jointNameToMatrixIndexFunction);
      }

      dynamicsMatrixCalculator.compute();
      dynamicsMatrixCalculator.getMassMatrix(massMatrix);
      dynamicsMatrixCalculator.getCoriolisMatrix(coriolisGravityMatrix);

      CommonOps_DDRM.mult(massMatrix, controllerDesiredQdd, controllerDesiredTau);
      CommonOps_DDRM.addEquals(controllerDesiredTau, coriolisGravityMatrix);

      if (estimateContactPosition.getBooleanValue())
      {
         contactParticleFilter.doControl();
         // TODO pack output
      }
      else
      {
         predefinedContactForceSolver.doControl();

         outputStatus.getEstimatedExternalForces().clear();
         YoFixedFrameSpatialVector[] estimatedExternalWrenches = predefinedContactForceSolver.getEstimatedExternalWrenches();

         int numberOfContactPoints = predefinedContactForceSolver.getNumberOfContactPoints() - (calculateRootJointWrench.getValue() ? 1 : 0);
         for (int i = 0; i < numberOfContactPoints; i++)
         {
            outputStatus.getEstimatedExternalForces().add().set(estimatedExternalWrenches[i].getLinearPart());
         }

         if(calculateRootJointWrench.getValue())
         {
            int lastIndex = predefinedContactForceSolver.getNumberOfContactPoints() - 1;
            outputStatus.getEstimatedRootJointWrench().getTorque().set(predefinedContactForceSolver.getEstimatedExternalWrenches()[lastIndex].getAngularPart());
            outputStatus.getEstimatedRootJointWrench().getForce().set(predefinedContactForceSolver.getEstimatedExternalWrenches()[lastIndex].getLinearPart());
         }
         else
         {
            outputStatus.getEstimatedRootJointWrench().getTorque().setToNaN();
            outputStatus.getEstimatedRootJointWrench().getForce().setToNaN();
         }
      }

      outputStatus.setSequenceId(outputStatus.getSequenceId() + 1);
      statusOutputManager.reportStatusMessage(outputStatus);
   }

   public void updateRobotConfigurationData(RobotConfigurationData robotConfigurationData)
   {
      this.robotConfigurationData.set(robotConfigurationData);
   }

   public void updateRobotDesiredConfigurationData(RobotDesiredConfigurationData robotDesiredConfigurationData)
   {
      this.robotDesiredConfigurationData.set(robotDesiredConfigurationData);
   }

   @Override
   public boolean isDone()
   {
      return false;
   }

   public FloatingJointBasics getRootJoint()
   {
      return rootJoint;
   }

   public OneDoFJointBasics[] getOneDoFJoints()
   {
      return oneDoFJoints;
   }

   private static void updateRobotState(RobotConfigurationData robotConfigurationData, FloatingJointBasics rootJoint, OneDoFJointBasics[] oneDoFJoints)
   {
      TFloatArrayList newJointAngles = robotConfigurationData.getJointAngles();
      TFloatArrayList newJointVelocities = robotConfigurationData.getJointVelocities();

      if(newJointAngles.size() != oneDoFJoints.length)
      {
         throw new RuntimeException("Received RobotConfigurationData packet with " + newJointAngles.size() + "joints, expected " + oneDoFJoints.length);
      }

      for (int i = 0; i < newJointAngles.size(); i++)
      {
         oneDoFJoints[i].setQ(newJointAngles.get(i));
         oneDoFJoints[i].setQd(newJointVelocities.get(i));
      }

      rootJoint.setJointConfiguration(robotConfigurationData.getRootOrientation(), robotConfigurationData.getRootTranslation());
      rootJoint.setJointLinearVelocity(robotConfigurationData.getPelvisLinearVelocity());
      rootJoint.setJointAngularVelocity(robotConfigurationData.getPelvisAngularVelocity());

      rootJoint.getPredecessor().updateFramesRecursively();
      rootJoint.updateFramesRecursively();
   }

   private static void updateRobotDesiredState(RobotDesiredConfigurationData desiredConfigurationData, DMatrixRMaj controllerDesiredQdd, ToIntFunction<String> jointNameToMatrixIndex)
   {
      CommonOps_DDRM.fill(controllerDesiredQdd, 0.0);
      desiredConfigurationData.getDesiredRootJointAngularAcceleration().get(0, controllerDesiredQdd);
      desiredConfigurationData.getDesiredRootJointLinearAcceleration().get(3, controllerDesiredQdd);

      RecyclingArrayList<JointDesiredOutputMessage> jointDesiredOutputList = desiredConfigurationData.getJointDesiredOutputList();
      for (int i = 0; i < jointDesiredOutputList.size(); i++)
      {
         String jointName = jointDesiredOutputList.get(i).getJointName().toString();
         int matrixIndex = jointNameToMatrixIndex.applyAsInt(jointName);

         controllerDesiredQdd.set(matrixIndex, 0, jointDesiredOutputList.get(i).getDesiredAcceleration());
      }
   }
}
