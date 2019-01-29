package us.ihmc.footstepPlanning.graphSearch.collision;

import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.HashMap;

public class FootstepNodeBodyCollisionDetector
{
   private final BoundingBoxCollisionDetector collisionDetector;
   private final HashMap<LatticeNode, BodyCollisionData> collisionDataHolder = new HashMap<>();

   public FootstepNodeBodyCollisionDetector(FootstepPlannerParameters parameters)
   {
      this.collisionDetector = new BoundingBoxCollisionDetector(parameters);
   }

   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      collisionDetector.setPlanarRegionsList(planarRegionsList);
      collisionDataHolder.clear();
   }

   public BodyCollisionData checkForCollision(LatticeNode node, double height)
   {
      if(collisionDataHolder.containsKey(node))
      {
         return collisionDataHolder.get(node);
      }
      else
      {
         collisionDetector.setBoxPose(node.getX(), node.getY(), height, node.getYaw());
         BodyCollisionData collisionData = collisionDetector.checkForCollision();
         collisionDataHolder.put(node, collisionData);
         return collisionData;
      }
   }
}