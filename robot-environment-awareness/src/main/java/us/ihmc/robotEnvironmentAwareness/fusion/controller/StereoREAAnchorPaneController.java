package us.ihmc.robotEnvironmentAwareness.fusion.controller;

import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Label;
import javafx.scene.control.Slider;
import javafx.scene.control.ToggleButton;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;
import us.ihmc.robotEnvironmentAwareness.ui.properties.ImageSegmentationParametersProperty;
import us.ihmc.robotEnvironmentAwareness.ui.properties.PlanarRegionPropagationParametersProperty;

public class StereoREAAnchorPaneController
{
   private JavaFXMessager messager;

   @FXML
   private Label computationTime;

   @FXML
   private ToggleButton enableREA;

   @FXML
   private Button clearREA;

   @FXML
   private Slider stereoBuffer;

   @FXML
   private Button showBuffer;

   @FXML
   private Button showProjection;

   @FXML
   private Slider superpixelSize;

   @FXML
   private Slider superpixelRuler;

   @FXML
   private Slider superpixelIterate;

   @FXML
   private ToggleButton enableConnectivity;

   @FXML
   private Slider superpixelMinimumElement;

   @FXML
   private Slider sparseLowerThreshold;

   @FXML
   private Slider sparseUpperThreshold;

   @FXML
   private Slider proximityThreshold;

   @FXML
   private Slider planarityThreshold;

   @FXML
   private CheckBox enableExtending;

   @FXML
   private CheckBox updateExtendedData;

   @FXML
   private Slider extendingDistanceThreshold;

   @FXML
   private Slider ExtendingRadius;

   private final ImageSegmentationParametersProperty imageSegmentationParametersProperty = new ImageSegmentationParametersProperty(this,
                                                                                                                                   "imageSegmentationParametersProperty");
   private final PlanarRegionPropagationParametersProperty planarRegionPropagationParametersProperty = new PlanarRegionPropagationParametersProperty(this,
                                                                                                                                                     "planarRegionPropagationParametersProperty");

   @FXML
   private Button runSREA;

   public void initialize(JavaFXMessager messager)
   {
      this.messager = messager;

      messager.bindBidirectional(LidarImageFusionAPI.EnableREA, enableREA.selectedProperty(), true);
      messager.bindBidirectional(LidarImageFusionAPI.ComputationTime, computationTime.textProperty(), false);
      messager.bindBidirectional(LidarImageFusionAPI.StereoBufferSize, stereoBuffer.valueProperty(), numberToIntegerConverter, true);

      imageSegmentationParametersProperty.bindBidirectionalPixelSize(superpixelSize.valueProperty());
      imageSegmentationParametersProperty.bindBidirectionalPixelRuler(superpixelRuler.valueProperty());
      imageSegmentationParametersProperty.bindBidirectionalIterate(superpixelIterate.valueProperty());
      imageSegmentationParametersProperty.bindBidirectionalEnableConnectivity(enableConnectivity.selectedProperty());
      imageSegmentationParametersProperty.bindBidirectionalMinElementSize(superpixelMinimumElement.valueProperty());
      messager.bindBidirectional(LidarImageFusionAPI.ImageSegmentationParameters, imageSegmentationParametersProperty, true);

      planarRegionPropagationParametersProperty.bindBidirectionalSparseThreshold(sparseLowerThreshold.valueProperty(), sparseUpperThreshold.valueProperty());
      planarRegionPropagationParametersProperty.bindBidirectionalProximityThreshold(proximityThreshold.valueProperty());
      planarRegionPropagationParametersProperty.bindBidirectionalPlanarityThreshold(planarityThreshold.valueProperty());
      planarRegionPropagationParametersProperty.bindBidirectionalEnableExtending(enableExtending.selectedProperty());
      planarRegionPropagationParametersProperty.bindBidirectionalUpdateExtendedData(updateExtendedData.selectedProperty());
      planarRegionPropagationParametersProperty.bindBidirectionalExtendingDistanceThreshold(extendingDistanceThreshold.valueProperty());
      planarRegionPropagationParametersProperty.bindBidirectionalExtendingRadiusThreshold(ExtendingRadius.valueProperty());
      messager.bindBidirectional(LidarImageFusionAPI.PlanarRegionPropagationParameters, planarRegionPropagationParametersProperty, true);
   }

   private final PropertyToMessageTypeConverter<Integer, Number> numberToIntegerConverter = new PropertyToMessageTypeConverter<Integer, Number>()
   {
      @Override
      public Integer convert(Number propertyValue)
      {
         return propertyValue.intValue();
      }

      @Override
      public Number interpret(Integer newValue)
      {
         return new Double(newValue.intValue());
      }
   };

   public void clearREA()
   {
      messager.submitMessage(LidarImageFusionAPI.ClearREA, true);
   }

   public void showBuffer()
   {
      messager.submitMessage(LidarImageFusionAPI.ShowStereoBuffer, true);
   }

   public void showProjection()
   {
      messager.submitMessage(LidarImageFusionAPI.ShowStereoBufferProjection, true);
   }

   public void runSREA()
   {
      messager.submitMessage(LidarImageFusionAPI.RunStereoREA, true);
   }
}