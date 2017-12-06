package us.ihmc.footstepPlanning.graphSearch;

import static junit.framework.TestCase.assertFalse;
import static junit.framework.TestCase.assertTrue;
import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.footstepPlanning.DefaultFootstepPlanningParameters;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.PlanarRegionBaseOfCliffAvoider;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = IntegrationCategory.IN_DEVELOPMENT)
public class PlanarRegionBaseOfCliffAvoiderTest
{
   private final boolean visualize = false;
   private final boolean doAsserts = true;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000000)
   public void testBaseOfCliffAvoiderWithSimpleQueriesOnABlock()
   {
      double stepHeight = 0.2;
      double boxSize = 1.0;
      double edgeOfBoxX = 1.0;

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(edgeOfBoxX + boxSize / 2.0, 0.0, 0.0);
      generator.addCubeReferencedAtBottomMiddle(boxSize, boxSize, stepHeight);
      generator.translate(0.0, 0.0, 0.001);
      generator.addRectangle(5.0, 5.0); // floor plane
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      YoVariableRegistry registry = new YoVariableRegistry("Test");
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      double epsilon = 1e-6;
      double minimumDistanceFromCliffBottom = 0.2 - epsilon;
      YoFootstepPlannerParameters parameters = new YoFootstepPlannerParameters(registry, new DefaultFootstepPlanningParameters()
      {
         @Override
         public double getCliffHeightToAvoid()
         {
            return 0.01;
         }

         @Override
         public double getMinimumDistanceFromCliffBottoms()
         {
            return minimumDistanceFromCliffBottom;
         }
      });

      double footLength = 0.2;
      double footWidth = 0.1;
      SideDependentList<ConvexPolygon2D> footPolygons = PlanningTestTools.createFootPolygons(footLength, footWidth);
      PlanarRegionBaseOfCliffAvoider avoider = new PlanarRegionBaseOfCliffAvoider(parameters, footPolygons);
      avoider.setPlanarRegions(planarRegionsList);

      SimulationConstructionSet scs = null;
      if (visualize)
      {
         scs = new SimulationConstructionSet(new Robot("TestRobot"));
         scs.addYoVariableRegistry(registry);
         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
         Graphics3DObject staticLinkGraphics = new Graphics3DObject();
         staticLinkGraphics.addCoordinateSystem(1.0);
         staticLinkGraphics.addPlanarRegionsList(planarRegionsList, YoAppearance.Green(), YoAppearance.Beige(), YoAppearance.Yellow(), YoAppearance.Orange());
         scs.addStaticLinkGraphics(staticLinkGraphics);
         scs.startOnAThread();
         ThreadTools.sleepForever();
      }

      double closestNodeDistanceToCliff = edgeOfBoxX - 0.5 * footLength - minimumDistanceFromCliffBottom;

      RobotSide footstepSide = RobotSide.LEFT;
      double x = closestNodeDistanceToCliff;
      double y = 0.0;
      FootstepNode node = new FootstepNode(x, y, 0.0, footstepSide);
      assertTrue(avoider.isNodeValid(node, null));

      x = closestNodeDistanceToCliff + FootstepNode.gridSizeXY;
      node = new FootstepNode(x, y, 0.0, footstepSide);
      assertFalse(avoider.isNodeValid(node, null));

      x = closestNodeDistanceToCliff - FootstepNode.gridSizeXY;
      node = new FootstepNode(x, y, 0.0, footstepSide);
      assertTrue(avoider.isNodeValid(node, null));
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(PlanarRegionBaseOfCliffAvoider.class, PlanarRegionBaseOfCliffAvoiderTest.class);
   }
}
