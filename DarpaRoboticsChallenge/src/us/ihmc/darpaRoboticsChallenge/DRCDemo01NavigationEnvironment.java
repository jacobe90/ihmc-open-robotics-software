package us.ihmc.darpaRoboticsChallenge;

import java.net.URL;
import java.util.ArrayList;
import java.util.Random;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point2d;
import javax.vecmath.Vector3d;

import us.ihmc.commonAvatarInterfaces.CommonAvatarEnvironmentInterface;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceTexture;
import us.ihmc.utilities.math.geometry.Box3d;
import us.ihmc.utilities.math.geometry.ConvexPolygon2d;

import com.yobotics.simulationconstructionset.ExternalForcePoint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.util.environments.SelectableObjectListener;
import com.yobotics.simulationconstructionset.util.ground.CombinedTerrainObject;
import com.yobotics.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import com.yobotics.simulationconstructionset.util.ground.TerrainObject;
import com.yobotics.simulationconstructionset.util.ground.RotatableConvexPolygonTerrainObject;

public class DRCDemo01NavigationEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject combinedTerrainObject;

   private final Random random = new Random(1989L);

   private static final double ROCKS_START_Y = 3.5;
   private static final double ROCK_PATH_LENGTH = 8.0;

   private static final int NUM_ROCKS = 80;
   private static final double MAX_ROCK_CENTROID_HEIGHT = 0.2;
   private static final int POINTS_PER_ROCK = 21;

   // chance unevenness of rocks
   private static final double MAX_ABS_XY_NORMAL_VALUE = 0.0;
   private static final double ROCK_FIELD_WIDTH = 2.0;
   private static final double ROCK_BOUNDING_BOX_WIDTH = 0.5;

   private static final boolean FULLY_RANDOM = true;    // Will do a neat grid if set to false;
   private static final int ROCKS_PER_ROW = 4;
   private static final boolean DIFFICULT_STEPPING_STONES = false;    // for path 8, if true creates an extension to the path with harder steps

   private boolean addLimboBar = false;

   // private static final double FLOOR_THICKNESS = 0.001;

   public DRCDemo01NavigationEnvironment()
   {
      combinedTerrainObject = new CombinedTerrainObject("Rocks with a wall");
      setUpPath1Rocks();
      setUpPath2SmallCones();
      setUpPath3RampsWithLargeBlocks();
      setUpPath4DRCTrialsTrainingWalkingCourse();
      setUpPath5NarrowDoor();
      setUpPath6Barriers();
      setUpPath7Stairs();
      setUpPath8RampsWithSteppingStones();

      setUpGround();

      conditionallyAddLimboBar();
   }

   private void setUpPath1Rocks()
   {
      addRocks();
   }

   private void setUpPath2SmallCones()
   {
      int numCones = 4;
      float initialOffset = 2.952f;
      float coneSeparation = 1.5f;
      float coneColorSeparateion = 0.1f;

      for (int i = 0; i < numCones; i++)
      {
         AppearanceDefinition cone1;
         AppearanceDefinition cone2;
         if (i % 2 == 0)
         {
            cone1 = YoAppearance.Green();
            cone2 = YoAppearance.Red();
         }
         else
         {
            cone1 = YoAppearance.Red();
            cone2 = YoAppearance.Green();
         }

         setUpCone(initialOffset + (i * coneSeparation) + coneColorSeparateion, -(initialOffset + (i * coneSeparation)), .25, .25, 0.5, cone1);
         setUpCone(initialOffset + (i * coneSeparation), -(initialOffset + (i * coneSeparation) + coneColorSeparateion), .25, .25, 0.45, cone2);
      }

   }

   private void setUpPath3RampsWithLargeBlocks()
   {
      AppearanceDefinition color = YoAppearance.DarkGray();

      // float rampHeight = 0.3f;

      float rampHeight = 0.625f;

      setUpRamp(5.0f, 0.0f, 2.0f, 3.0f, rampHeight, color);
      setUpWall(7.0f, 0.0f, .5f, 1.0f, rampHeight, 0, color);

      setUpWall(7.75f, 0.0f, 2f, .5f, rampHeight, 0, color);
      setUpWall(8.5f, 0f, .5f, .75f, rampHeight - 0.1, 0, color);

      setUpWall(8.5f, .75f, .5f, .75f, rampHeight, 0, color);

      setUpWall(8.5f, -0.66f, .25f, 1f, rampHeight, 0, color);

      setUpWall(8.5f, -1.045f, .25f, 1f, rampHeight, 0, color);



      setUpWall(9.25f, 0f, 2.0f, 0.5f, rampHeight, 0, color);
      setUpRamp(11f, 0f, 2.0f, -3.0f, rampHeight, color);

      // Do this for a long ramp for testing:
      // rampHeight = 1.0f;
      // setUpRamp(10.1, 0.0f, 2.0f, 20.0f, rampHeight, color);
   }

   private void setUpPath4DRCTrialsTrainingWalkingCourse()
   {
      // TODO: Finish course
      double courseAngle = 45.0;
      double startDistance = 4.0;
      AppearanceDefinition color = YoAppearance.Gray();

      final double sectionLength = 2.4;

      for (int i = 1; i <= 4; i = i + 2)
      {
         double rampLength = sectionLength / 4;
         double rampAngle = Math.toRadians(15);
         double rampHeight = rampLength * Math.tan(rampAngle);
         double rampCenter = startDistance + rampLength * (i - 1) + rampLength / 2;
         double[] newPoint = rotateAroundOrigin(rampCenter, 0, courseAngle);
         setUpRotatedRamp(newPoint[0], newPoint[1], sectionLength, rampLength, rampHeight, courseAngle, color);
         
         double rampDownCenter = startDistance + rampLength * (i) + rampLength / 2;
         newPoint = rotateAroundOrigin(rampDownCenter, 0, courseAngle);
         setUpRotatedRamp(newPoint[0], newPoint[1], sectionLength, -rampLength, rampHeight, courseAngle, color);
      }

//    setUpCinderBlock(0,0,16,8,6,0,YoAppearance.DarkGray());
//
//    int numberOfStepOvers = 3;
//    double heightIncrease = 0.05;
//    double spacing = 1.0;
//
//    double barrierWidth = 3.0;
//
//    
//    for (int i = 0; i < numberOfStepOvers; i++)
//    {
//        setUpWall(newPoint[0], newPoint[1], barrierWidth, 0.15, heightIncrease * (i + 1), courseAngle, color);
//    }
   }

   private void setUpPath5NarrowDoor()
   {
      AppearanceDefinition color = YoAppearance.DarkGray();

      // angled Door
      // door1
      setUpWall(0.769f, -9.293f, 0.973f, 0.157f, 2.5f, -115.0f, color);

      // door2
      setUpWall(-.642f, -8.635f, 0.973f, 0.157f, 2.54f, -115.0f, color);

      // box2
      setUpWall(-0.485f, -6.573f, 0.5f, 0.5f, 1.0f, -45, color);

      // box1
      setUpWall(0.515f, -4.972f, 0.5f, 0.5f, 1.0f, -110.0f, color);

   }

   private void setUpPath6Barriers()
   {
      AppearanceDefinition color = YoAppearance.DarkGray();
      double courseAngle = -135.0;
      int numberOfStepOvers = 8;
      double heightIncrease = 0.05;
      double startDistance = 4.0;
      double spacing = 1.0;

      double barrierWidth = 3.0;
      double platformWidth = 0.8;

      for (int i = 0; i < numberOfStepOvers; i++)
      {
         double[] newPoint = rotateAroundOrigin(startDistance + (i * spacing), 0, courseAngle);
         setUpWall(newPoint[0], newPoint[1], barrierWidth, 0.15, heightIncrease * (i + 1), courseAngle, color);
      }

      for (int i = 0; i < numberOfStepOvers; i++)
      {
         double[] newPoint = rotateAroundOrigin(startDistance + (i * spacing), (barrierWidth - platformWidth) / 2.0 + 0.001, courseAngle);
         setUpWall(newPoint[0], newPoint[1], platformWidth, 0.4 * spacing, heightIncrease * (i + 1) + 0.001, courseAngle, color);
      }

   }

   private void setUpPath7Stairs()
   {
      AppearanceDefinition color = YoAppearance.DarkGray();
      double courseAngle = 135;
      int numberOfSteps = 3;
      double rise = 0.2;
      double startDistance = 4.0;
      double run = 0.4;

      for (int i = 0; i < numberOfSteps; i++)
      {
         double[] newPoint = rotateAroundOrigin(startDistance + (i * run), 0, courseAngle);
         setUpWall(newPoint[0], newPoint[1], 3.0, run, rise * (i + 1), courseAngle, color);
      }

      {
         double[] newPoint = rotateAroundOrigin(startDistance + (numberOfSteps * run), 0, courseAngle);
         setUpWall(newPoint[0], newPoint[1], 3.0, run, rise * (numberOfSteps - 1 + 1), courseAngle, color);
      }

      for (int i = 1; i < numberOfSteps + 1; i++)
      {
         double offset = numberOfSteps * run;
         double[] newPoint = rotateAroundOrigin(offset + startDistance + (i * run), 0, courseAngle);
         setUpWall(newPoint[0], newPoint[1], 3.0, run, rise * (-i + numberOfSteps + 1), courseAngle, color);
      }
   }

   private void setUpPath8RampsWithSteppingStones()
   {
      AppearanceDefinition color = YoAppearance.DarkGray();

      float rampHeight = 0.3f;

      // ramp up and landing
      setUpRamp(-5.0f, 0.0f, 3.0f, -3.0f, rampHeight, color);
      setUpWall(-7.0f, 0.0f, 3.0f, 1.0f, rampHeight, 0, color);

      // simple stepping stones, centered at x=-0.75m
      setUpWall(-7.75f, -0.5f, 0.5f, 0.5f, rampHeight, 0, color);
      setUpWall(-8.25f, -1.0f, 0.5f, 0.5f, rampHeight, 0, color);
      setUpWall(-8.75f, -0.5f, 0.5f, 0.5f, rampHeight, 0, color);
      setUpWall(-9.25f, -1.0f, 0.5f, 0.5f, rampHeight, 0, color);
      setUpWall(-8.75f, -0.5f, 0.5f, 0.5f, rampHeight, 0, color);
      setUpWall(-9.25f, -1.0f, 0.5f, 0.5f, rampHeight, 0, color);
      setUpWall(-9.75f, -0.5f, 0.5f, 0.5f, rampHeight, 0, color);

      // qualification stepping stones, centered along x=0.75m
      setUpWall(-8.0f, 1.0f, 0.5f, 0.5f, rampHeight, 0, color);
      setUpWall(-8.5f, 0.5f, 0.5f, 0.5f, rampHeight, 0, color);
      setUpWall(-9.3f, 1.0f, 0.5f, 0.5f, rampHeight, 0, color);

      // middle landing
      setUpWall(-10.5f, 0.0f, 3.0f, 1.0f, rampHeight, 0, color);

      if (DIFFICULT_STEPPING_STONES)
      {
         // more difficult stepping stones
         setUpWall(-11.6f, -0.35f, 0.5f, 0.5f, rampHeight, 0, color);
         setUpWall(-12.2f, 0.35f, 0.5f, 0.5f, rampHeight, 0, color);
         setUpWall(-13.1f, 0.15f, 0.5f, 0.5f, rampHeight, 0, color);
         setUpWall(-14f, 0.95f, 0.5f, 0.5f, rampHeight, 0, color);

         // landing and ramp down
         setUpWall(-15.5f, 0.5f, 2.0f, 1.0f, rampHeight, 0, color);
         setUpRamp(-17.5f, 0.5f, 2.0f, 3.0f, rampHeight, color);
      }
      else
      {
         setUpRamp(-12.5f, 0.0f, 3.0f, 3.0f, rampHeight, color);
      }

      // Do this for a long ramp for testing:
      // rampHeight = 1.0f;
      // setUpRamp(10.1, 0.0f, 2.0f, 20.0f, rampHeight, color);
   }

   private void setUpGround()
   {
      // AppearanceDefinition app = YoAppearance.Gray();

      // center
      // setUpCone(0, 0, 1.5, 1.5, FLOOR_THICKNESS, app);

      // filler

      // setUpCone(0, 0, 12.5, 14.5, 0.0001, app);

      // setUpCone(0, 0, 10, 12, 0.005, YoAppearance.Brown());

      URL fileURL = DRCDemo01NavigationEnvironment.class.getResource("Textures/ground2.png");
      YoAppearanceTexture texture = new YoAppearanceTexture(fileURL);

      Transform3D location = new Transform3D();
      location.setTranslation(new Vector3d(0, 0, -0.5));

      RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3d(location, 45, 45, 1), texture);
      combinedTerrainObject.addTerrainObject(newBox);
      RotatableBoxTerrainObject newBox2 = new RotatableBoxTerrainObject(new Box3d(location, 200, 200, 0.75), YoAppearance.DarkGray());
      combinedTerrainObject.addTerrainObject(newBox2);

   }

   private void conditionallyAddLimboBar()
   {
      if (addLimboBar)
      {
         double height = 1;
         double width = 1.5;
         AppearanceDefinition color = YoAppearance.DarkGray();

         setUpWall(1, width / 2, 0.125, 0.125, height, 0, color);
         setUpWall(1, -width / 2, 0.125, 0.125, height, 0, color);

         combinedTerrainObject.getLinkGraphics().translate(0, width / 2, height);
         combinedTerrainObject.getLinkGraphics().addCube(0.125, width, 0.125, color);
         combinedTerrainObject.getLinkGraphics().translate(0, -width / 2, -height);
      }
   }

   private void addRocks()
   {
      for (int i = 0; i < NUM_ROCKS; i++)
      {
         double centroidHeight = random.nextDouble() * MAX_ROCK_CENTROID_HEIGHT;
         Vector3d normal = generateRandomUpFacingNormal();

         double[] approximateCentroid = generateRandomApproximateCentroid(i);

         double[][] vertices = generateRandomRockVertices(approximateCentroid[0], approximateCentroid[1]);

         addRock(normal, centroidHeight, vertices);
      }
   }

   private double[] rotateAroundOrigin(double x, double y, double angdeg)
   {
      double[] newPoint = new double[2];
      newPoint[0] = x * Math.cos(Math.toRadians(angdeg)) - y * Math.sin(Math.toRadians(angdeg));
      newPoint[1] = y * Math.cos(Math.toRadians(angdeg)) + x * Math.sin(Math.toRadians(angdeg));

      return newPoint;
   }

   private double[] generateRandomApproximateCentroid(int position)
   {
      double[] approximateCentroid = new double[2];

      if (FULLY_RANDOM)
      {
         approximateCentroid[0] = random.nextDouble() * ROCK_FIELD_WIDTH - ROCK_FIELD_WIDTH / 2.0;
         approximateCentroid[1] = random.nextDouble() * ROCK_PATH_LENGTH + ROCKS_START_Y;

      }
      else
      {
         int row = position / ROCKS_PER_ROW;
         int rows = NUM_ROCKS / ROCKS_PER_ROW;
         double distancePerRow = ROCK_PATH_LENGTH / ((double) rows - 1);
         approximateCentroid[1] = ROCKS_START_Y + distancePerRow * row;

         int positionOnRow = position - row * ROCKS_PER_ROW;
         approximateCentroid[0] = ROCK_FIELD_WIDTH * ((double) positionOnRow) / ((double) ROCKS_PER_ROW) - ROCK_FIELD_WIDTH / 2.0;
      }

      return approximateCentroid;
   }

   private Vector3d generateRandomUpFacingNormal()
   {
      double normalX = random.nextDouble() * (2.0 * MAX_ABS_XY_NORMAL_VALUE) - MAX_ABS_XY_NORMAL_VALUE;
      double normalY = random.nextDouble() * (2.0 * MAX_ABS_XY_NORMAL_VALUE) - MAX_ABS_XY_NORMAL_VALUE;
      Vector3d normal = new Vector3d(normalX, normalY, 1.0);

      return normal;
   }

   private double[][] generateRandomRockVertices(double approximateCentroidX, double approximateCentroidY)
   {
      double[][] vertices = new double[POINTS_PER_ROCK][2];

      for (int j = 0; j < POINTS_PER_ROCK; j++)
      {
         vertices[j][0] = random.nextDouble() * ROCK_BOUNDING_BOX_WIDTH + approximateCentroidX - ROCK_BOUNDING_BOX_WIDTH / 2.0;
         vertices[j][1] = random.nextDouble() * ROCK_BOUNDING_BOX_WIDTH + approximateCentroidY - ROCK_BOUNDING_BOX_WIDTH / 2.0;
      }

      return vertices;
   }

   private void addRock(Vector3d normal, double centroidHeight, double[][] vertices)
   {
      AppearanceDefinition color = YoAppearance.DarkGray();

      ArrayList<Point2d> vertexPoints = new ArrayList<Point2d>();

      for (double[] point : vertices)
      {
         Point2d point2d = new Point2d(point);
         vertexPoints.add(point2d);
      }

      ConvexPolygon2d convexPolygon = new ConvexPolygon2d(vertexPoints);
      RotatableConvexPolygonTerrainObject rock = new RotatableConvexPolygonTerrainObject(normal, convexPolygon, centroidHeight, color);
      this.combinedTerrainObject.addTerrainObject(rock);
   }

   private void setUpWall(double x, double y, double width, double length, double height, double yawDegrees, AppearanceDefinition app)
   {
      Transform3D location = new Transform3D();
      location.rotZ(Math.toRadians(yawDegrees));

      location.setTranslation(new Vector3d(x, y, height / 2));
      RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3d(location, length, width, height), app);
      combinedTerrainObject.addTerrainObject(newBox);
   }

   private void setUpCone(double x, double y, double bottomWidth, double topWidth, double height, AppearanceDefinition app)
   {
      combinedTerrainObject.addCone(x, y, bottomWidth, topWidth, height, app);
   }

   private void setUpRamp(double x, double y, double width, double length, double height, AppearanceDefinition app)
   {
      combinedTerrainObject.addRamp(x - length / 2, y - width / 2, x + length / 2, y + width / 2, height, app);
   }

   private void setUpRotatedRamp(double xCenter, double yCenter, double width, double run, double rise, double yawDegreesAboutCenter, AppearanceDefinition app)
   {
      combinedTerrainObject.addRotatedRamp(xCenter - run / 2, yCenter - width / 2, xCenter + run / 2, yCenter + width / 2, rise, yawDegreesAboutCenter, app);
   }

   private void setUpCinderBlock(double x, double y, double width, double length, double height, double yawDegrees, AppearanceDefinition app)
   {
      // TODO: FINISH
      // ramp
      combinedTerrainObject.addRamp(x - length / 2, y - width / 2, x + length / 2, y + width / 2, height, app);

      // wall
      Transform3D location = new Transform3D();
      location.rotZ(Math.toRadians(yawDegrees));

      location.setTranslation(new Vector3d(x, y, height / 2));
      RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3d(location, length, width, height), app);
      combinedTerrainObject.addTerrainObject(newBox);
   }

   public TerrainObject getTerrainObject()
   {
      return combinedTerrainObject;
   }

   public ArrayList<Robot> getEnvironmentRobots()
   {
      return new ArrayList<Robot>();
   }

   public void createAndSetContactControllerToARobot()
   {
      // TODO Auto-generated method stub

   }

   public void addContactPoints(ExternalForcePoint[] externalForcePoints)
   {
      // TODO Auto-generated method stub

   }

   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
      // TODO Auto-generated method stub

   }

}
