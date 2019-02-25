package us.ihmc.atlas.heightForBalance;

import afu.org.checkerframework.checker.oigj.qual.O;
import us.ihmc.avatar.heightForBalanceTest.AvatarHeightForBalanceTest;

import org.junit.Test;
import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.*;
import us.ihmc.atlas.straightLegWalking.AtlasStraightLegWalkingTest.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.capturePoint.heightForBalance.HeightForBalanceParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.*;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationGains;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class AtlasHeightForBalanceTest extends AvatarHeightForBalanceTest
{
   private final RobotTarget target = RobotTarget.SCS;
   private final AtlasRobotModel atlasRobotModel = new MyAtlasRobotModel();
   private final AtlasRobotModel atlasRobotModelNormal = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, target, false);
   private final boolean useNormalRobotModel = true;
   private final double privilegedAngleWhenStraight = 1.177;


   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 300000)
   public void testPushStanding() throws Exception
   {
      // No-height max recoverable percentWeight: 0.57
      percentWeight = 0.48;//0.217;//0.212;//0.233;//0.241;//0.218;//.206;//0.221;  7%
      super.testPushStanding();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 300000)
   public void testPushFrontalInSwing() throws Exception
   {
      // No-height max recoverable percentWeight: 1.35
      percentWeight = 1.42;
      super.testPushFrontalInSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 300000)
   public void testPushBackInSwing() throws Exception
   {
      // No-height max recoverable percentWeight: 0.72
      percentWeight = 0.80;
      super.testPushBackInSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 300000)
   public void testPushRightInSwing() throws Exception
   {
      // No-height max recoverable percentWeight: 0.55         WORSE!
      percentWeight = 0.48;
      super.testPushRightInSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 300000)
   public void testPushLeftInSwing() throws Exception
   {
      // No-height max recoverable percentWeight: 0.34
      percentWeight = 0.40;
      super.testPushLeftInSwing();
   }

   @Override
   public double getNominalHeight()
   {
      return 0.9;
   }

   @Override
   public double getSlowTransferDuration()
   {
      return 0.15;
   }

   @Override
   public double getSlowSwingDuration()
   {
      return 0.6;
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      if(useNormalRobotModel)
         return atlasRobotModelNormal;
      else
         return new HeightForBalanceModel();
   }

   @Override
   protected double getSizeScale()
   {
      return 1.0;
   }

   @Override
   public String getSimpleRobotName()
   {
      return "Atlas";
   }

   private class HeightForBalanceModel extends AtlasRobotModel
   {
      public HeightForBalanceModel() { super(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);}

      @Override
      public WalkingControllerParameters getWalkingControllerParameters()
      {
         return new HeightForBalanceWalkingControllerParameters(getJointMap(),getContactPointParameters());
      }
   }

   private class HeightForBalanceWalkingControllerParameters extends AtlasWalkingControllerParameters
   {
      private final AtlasJointMap jointMap;
      private final AtlasContactPointParameters contactPointParameters;
      public HeightForBalanceWalkingControllerParameters(AtlasJointMap jointmap, AtlasContactPointParameters contactPointParameters)
      {
         super(RobotTarget.SCS,jointmap,contactPointParameters);
         this.jointMap=jointmap;
         this.contactPointParameters=contactPointParameters;
      }
      @Override
      public boolean useHeightForBalanceController()
      {
         return  true;
      }
   }

   private class MyAtlasRobotModel extends AtlasRobotModel
   {
      public MyAtlasRobotModel()
      {
         super(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
      }

      @Override
      public WalkingControllerParameters getWalkingControllerParameters()
      {
         return new AtlasHeightForBalanceTest.TestWalkingControllerParameters(getJointMap(), getContactPointParameters());
      }
   }

   private class TestWalkingControllerParameters extends AtlasWalkingControllerParameters
   {
      private final AtlasJointMap jointMap;
      private final AtlasContactPointParameters contactPointParameters;

      public TestWalkingControllerParameters(AtlasJointMap jointMap, AtlasContactPointParameters contactPointParameters)
      {
         super(RobotTarget.SCS, jointMap, contactPointParameters);

         this.jointMap = jointMap;
         this.contactPointParameters = contactPointParameters;
      }

      @Override
      public double getMaxICPErrorBeforeSingleSupportX()
      {
         return 0.04;
      }

      @Override
      public double getMaxICPErrorBeforeSingleSupportY()
      {
         return 0.02;
      }

      @Override
      public boolean controlHeightWithMomentum()
      {
         return true;
      }

      @Override
      public boolean applySecondaryJointScaleDuringSwing()
      {
         return true;
      }

      @Override
      public boolean allowDisturbanceRecoveryBySpeedingUpSwing()
      {
         return  false;
      }

      @Override
      public LegConfigurationParameters getLegConfigurationParameters()
      {
         return new AtlasHeightForBalanceTest.TestLegConfigurationParameters();
      }

      @Override
      public MomentumOptimizationSettings getMomentumOptimizationSettings()
      {
         return new AtlasHeightForBalanceTest.TestMomentumOptimizationSettings(jointMap, contactPointParameters.getNumberOfContactableBodies());
      }

      @Override
      public ICPOptimizationParameters getICPOptimizationParameters()
      {
         return new AtlasHeightForBalanceTest.TestICPOptimizationParameters();
      }

   }

   private class TestLegConfigurationParameters extends AtlasLegConfigurationParameters
   {
      public TestLegConfigurationParameters()
      {
         super(false);
      }

      @Override
      public boolean attemptToStraightenLegs()
      {
         return false;
      }

      @Override
      public LegConfigurationGains getBentLegGains()
      {
         LegConfigurationGains gains = new LegConfigurationGains();
         gains.setJointSpaceKp(60.0);
         gains.setJointSpaceKd(6.0);

         return gains;
      }


      @Override
      public LegConfigurationGains getStraightLegGains()
      {
         LegConfigurationGains gains = new LegConfigurationGains();
         //gains.setJointSpaceKp(1000.0);
         gains.setActuatorSpaceKp(80);
         gains.setJointSpaceKd(5.0);

         return gains;
      }

      @Override
      public double getAccelerationForSupportKneeStraightening()
      {
         return 10.0;
      }

      @Override
      public double getSupportKneeCollapsingDurationFractionOfStep()
      {
         return 0.5;
         //      return 1.0;
         //      return 0.1; // for big step down
      }

      @Override
      public double getKneeAngleWhenStraight()
      {
         return privilegedAngleWhenStraight;
      }

      @Override
      public double getKneeAngleWhenBracing()
      {
         return 0.4;
      }

      @Override
      public double getKneeAngleWhenExtended()
      {
         return 0.0;
      }

      @Override
      public double getLegPrivilegedLowWeight()
      {
         return 5.0;
      }

      @Override
      public double getLegPrivilegedMediumWeight()
      {
         return 50.0;
      }

      @Override
      public double getLegPrivilegedHighWeight()
      {
         return 150.0;
      }

   }

   private class TestICPOptimizationParameters extends AtlasICPOptimizationParameters
   {
      public TestICPOptimizationParameters()
      {
         super(false);
      }

      @Override
      public boolean allowStepAdjustment()
      {
         return false;
      }
   }

   private class TestMomentumOptimizationSettings extends AtlasMomentumOptimizationSettings
   {
      public TestMomentumOptimizationSettings(AtlasJointMap jointMap, int numberOfContactableBodies)
      {
         super(jointMap, numberOfContactableBodies);
      }

      @Override
      public double getJointAccelerationWeight()
      {
         return 0.05;
      }
   }

}