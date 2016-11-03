package us.ihmc.darpaRoboticsChallenge.environment;

import java.util.List;

import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.util.environments.SelectableObjectListener;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class DRCTrialsTrainingWalkingCourseEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D combinedTerrainObject3D;


   public DRCTrialsTrainingWalkingCourseEnvironment()
   {
      combinedTerrainObject3D = new CombinedTerrainObject3D(getClass().getSimpleName());
      combinedTerrainObject3D.addTerrainObject(DRCDemo01NavigationEnvironment.setUpPath4DRCTrialsTrainingWalkingCourse("Path 4 Walking Course"));
      combinedTerrainObject3D.addTerrainObject(DRCDemo01NavigationEnvironment.setUpGround("Ground"));
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return combinedTerrainObject3D;
   }

   @Override
   public List<? extends Robot> getEnvironmentRobots()
   {
      return null;
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
   }

   @Override
   public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
   {
   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
   }
}
