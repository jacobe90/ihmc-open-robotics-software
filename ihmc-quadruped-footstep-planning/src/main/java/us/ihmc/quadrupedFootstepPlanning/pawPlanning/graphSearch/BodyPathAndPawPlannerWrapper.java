package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch;

import controller_msgs.msg.dds.QuadrupedGroundPlaneMessage;
import org.apache.commons.math3.util.Precision;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlanner;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanner;
import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.*;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawPlannerParametersReadOnly;
import us.ihmc.quadrupedFootstepPlanning.pathPlanning.WaypointsForPawPlanner;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

import java.util.ArrayList;
import java.util.List;

public class BodyPathAndPawPlannerWrapper implements BodyPathAndPawPlanner
{
   private static final boolean DEBUG = true;

   private static final double defaultTimeout = 5.0;
   private static final double defaultBestEffortTimeout = 0.0;

   protected final YoVariableRegistry registry;

   private final YoDouble timeout;
   private final YoDouble bestEffortTimeout;

   private final YoBoolean hasPath;
   private final YoDouble timeSpentBeforePawPlanner;
   private final YoDouble timeSpentInPawPlanner;
   private final YoEnum<PawPlanningResult> yoResult;

   protected final BodyPathPlanner bodyPathPlanner = new WaypointDefinedBodyPathPlanner();
   protected WaypointsForPawPlanner waypointPathPlanner;
   protected PawPlanner pawPlanner;

   private PlanarRegionsList planarRegionsList;

   private final boolean visualizing;
   private static final int bodyPathPointsForVisualization = 100;
   private final List<YoFramePoint3D> bodyPathPoints = new ArrayList<>();

   public BodyPathAndPawPlannerWrapper(String prefix, PawPlannerParametersReadOnly parameters, YoVariableRegistry parentRegistry,
                                       YoGraphicsListRegistry graphicsListRegistry)
   {
      registry = new YoVariableRegistry(prefix + getClass().getSimpleName());

      timeout = new YoDouble("timeout", registry);
      bestEffortTimeout = new YoDouble("bestEffortTimeout", registry);

      hasPath = new YoBoolean("hasPath", registry);
      timeSpentBeforePawPlanner = new YoDouble("timeSpentBeforePawPlanner", registry);
      timeSpentInPawPlanner = new YoDouble("timeSpentInPawPlanner", registry);
      yoResult = new YoEnum<>("planningResult", registry, PawPlanningResult.class);

      timeout.set(defaultTimeout);
      bestEffortTimeout.set(defaultBestEffortTimeout);
      visualizing = graphicsListRegistry != null;
      if (visualizing)
      {
         setupVisualization(prefix, graphicsListRegistry, registry);
      }

      parentRegistry.addChild(registry);
   }

   private void setupVisualization(String prefix, YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry registry)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList(prefix + "VisGraph");

      for (int i = 0; i < bodyPathPointsForVisualization; i++)
      {
         YoFramePoint3D point = new YoFramePoint3D(prefix + "BodyPathPoint" + i, ReferenceFrame.getWorldFrame(), registry);
         point.setToNaN();
         bodyPathPoints.add(point);
         YoGraphicPosition pointVisualization = new YoGraphicPosition(prefix + "BodyPathPoint" + i, point, 0.02, YoAppearance.Yellow());
         yoGraphicsList.add(pointVisualization);
      }

      graphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
   }

   @Override
   public WaypointsForPawPlanner getWaypointPathPlanner()
   {
      return waypointPathPlanner;
   }

   @Override
   public PawPlanner getPawPlanner()
   {
      return pawPlanner;
   }

   @Override
   public void setStart(PawPlannerStart start)
   {
      waypointPathPlanner.setInitialBodyPose(start.getTargetPose());

      pawPlanner.setStart(start);

      hasPath.set(false);
   }

   @Override
   public void setGoal(PawPlannerGoal goal)
   {
      waypointPathPlanner.setGoal(goal);

      pawPlanner.setGoal(goal);

      hasPath.set(false);
   }

   @Override
   public void setTimeout(double timeout)
   {
      this.timeout.set(timeout);
   }

   @Override
   public void setBestEffortTimeout(double timeout)
   {
      this.bestEffortTimeout.set(timeout);
   }

   @Override
   public void setGroundPlane(QuadrupedGroundPlaneMessage message)
   {
   }

   @Override
   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      waypointPathPlanner.setPlanarRegionsList(planarRegionsList);
      pawPlanner.setPlanarRegionsList(planarRegionsList);
      this.planarRegionsList = planarRegionsList;
   }

   @Override
   public double getPlanningDuration()
   {
      return timeSpentBeforePawPlanner.getDoubleValue() + timeSpentInPawPlanner.getDoubleValue();
   }

   @Override
   public void setPlanningHorizonLength(double planningHorizon)
   {
      pawPlanner.setPlanningHorizonLength(planningHorizon);
      hasPath.set(false);
   }

   @Override
   public PawPlanningResult planPath()
   {
      long startTime = System.currentTimeMillis();

      PawPlanningResult pathResult = waypointPathPlanner.planWaypoints();

      if (pathResult == PawPlanningResult.PLANNER_FAILED)
      {
         double seconds = (System.currentTimeMillis() - startTime) / 1000.0;
         timeSpentBeforePawPlanner.set(seconds);
         timeSpentInPawPlanner.set(0.0);
         yoResult.set(pathResult);
         return yoResult.getEnumValue();
      }

      List<Point3D> waypoints = waypointPathPlanner.getWaypoints();

      if (waypoints.size() < 2)
      {
//         if (parameters.getReturnBestEffortPlan())
//         {
//            waypointPathPlanner.computeBestEffortPlan(pawPlanner.getPlanningHorizonLength());
//         }
//         else
//         {
            double seconds = (System.currentTimeMillis() - startTime) / 1000.0;
            timeSpentBeforePawPlanner.set(seconds);
            timeSpentInPawPlanner.set(0.0);
            yoResult.set(PawPlanningResult.PLANNER_FAILED);
            return yoResult.getEnumValue();
//         }
      }

      bodyPathPlanner.setWaypoints(waypoints);
      bodyPathPlanner.compute();

      if (visualizing)
      {
         updateBodyPathVisualization();
      }

      double seconds = (System.currentTimeMillis() - startTime) / 1000.0;
      timeSpentBeforePawPlanner.set(seconds);

      hasPath.set(true);

      yoResult.set(PawPlanningResult.SUB_OPTIMAL_SOLUTION);
      return yoResult.getEnumValue();
   }

   @Override
   public PawPlanningResult plan()
   {
      if (!hasPath.getBooleanValue())
      {
         PawPlanningResult pathResult = planPath();
         if (!pathResult.validForExecution())
            return pathResult;
      }

      double bestEffortTimeout = this.bestEffortTimeout.getDoubleValue() - timeSpentBeforePawPlanner.getDoubleValue();
      pawPlanner.setTimeout(timeout.getDoubleValue() - timeSpentBeforePawPlanner.getDoubleValue());
      pawPlanner.setBestEffortTimeout(bestEffortTimeout);

      long startTime = System.currentTimeMillis();
      yoResult.set(pawPlanner.plan());
      double seconds = (System.currentTimeMillis() - startTime) / 1000.0;
      timeSpentInPawPlanner.set(seconds);

      if (DEBUG)
      {
         LogTools.info("Visibility graph with A* planner finished. Result: " + yoResult.getEnumValue());
         System.out.println("   Finished planning body path after " + Precision.round(timeSpentBeforePawPlanner.getDoubleValue(), 2) + " seconds.");
         System.out.println("   Finished planning paw path after " + Precision.round(timeSpentInPawPlanner.getDoubleValue(), 2) + " seconds.");
      }

      return yoResult.getEnumValue();
   }

   private void updateBodyPathVisualization()
   {
      Pose2D tempPose = new Pose2D();
      for (int i = 0; i < bodyPathPointsForVisualization; i++)
      {
         double percent = (double) i / (double) (bodyPathPointsForVisualization - 1);
         bodyPathPlanner.getPointAlongPath(percent, tempPose);
         Point3D position = new Point3D();
         position.set(tempPose.getPosition());
         Point3DReadOnly projectedPoint = PlanarRegionTools.projectPointToPlanesVertically(position, planarRegionsList);
         if (projectedPoint != null)
         {
            bodyPathPoints.get(i).set(projectedPoint);
         }
         else
         {
            bodyPathPoints.get(i).setToNaN();
         }
      }
   }

   @Override
   public void cancelPlanning()
   {
      pawPlanner.cancelPlanning();
   }

   @Override
   public PawPlan getPlan()
   {
      return pawPlanner.getPlan();
   }

   @Override
   public BodyPathPlan getPathPlan()
   {
      return bodyPathPlanner.getPlan();
   }

//   @Override
//   public ListOfStatistics getPlannerStatistics()
//   {
//      listOfStatistics.clear();
//
//      listOfStatistics.addStatistics(waypointPathPlanner.getPlannerStatistics());
//
//      return listOfStatistics;
//   }
}
