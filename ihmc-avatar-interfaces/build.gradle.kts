plugins {
   id("us.ihmc.ihmc-build") version "0.20.1"
   id("us.ihmc.ihmc-ci") version "5.3"
   id("us.ihmc.ihmc-cd") version "1.14"
   id("us.ihmc.log-tools") version "0.3.1"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("com.martiansoftware:jsap:2.1")
   api("org.apache.commons:commons-lang3:3.8.1")
   api("org.apache.poi:poi:3.15") // I/O library for xls files.
   api("us.ihmc.thirdparty.jinput:jinput:200128")
   api("org.ejml:core:0.30")
   api("org.ejml:dense64:0.30")
   api("org.boofcv:geo:0.24.1")
   api("org.jfree:jfreechart:1.0.17")
   api("com.github.quickhull3d:quickhull3d:1.0.0")
   api("com.github.wendykierp:JTransforms:3.1")
   api("org.reflections:reflections:0.9.10")

   api("us.ihmc:euclid:0.12.2")
   api("us.ihmc:ihmc-yovariables:0.4.0")
   api("us.ihmc:ihmc-commons:0.26.6")
   api("us.ihmc:ihmc-jmonkey-engine-toolkit:0.14.0")
   api("us.ihmc:simulation-construction-set:0.14.0")
   api("us.ihmc:ihmc-graphics-description:0.14.1")
   api("us.ihmc:robot-environment-awareness:source")
   api("us.ihmc:robot-environment-awareness-visualizers:source")
   api("us.ihmc:ihmc-ros-tools:source")
   api("us.ihmc:ihmc-common-walking-control-modules:source")
   api("us.ihmc:ihmc-robot-data-logger:0.15.0")
   api("us.ihmc:ihmc-model-file-loader:source")
   api("us.ihmc:ihmc-sensor-processing:source")
   api("us.ihmc:ihmc-communication:source")
   api("us.ihmc:ihmc-humanoid-behaviors:source")
   api("us.ihmc:ihmc-humanoid-robotics:source")
   api("us.ihmc:ihmc-perception:source")
   api("us.ihmc:ihmc-whole-body-controller:source")
   api("us.ihmc:ihmc-state-estimation:source")
   api("us.ihmc:ihmc-java-toolkit:source")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ihmc-robot-behaviors:source")
   api("us.ihmc:ihmc-mocap:source")
   api("us.ihmc:ihmc-robot-models:source")
   api("us.ihmc:ihmc-robot-models-visualizers:source")
   api("us.ihmc:ihmc-simulation-toolkit:source")
   api("us.ihmc:ihmc-robot-data-visualizer:source")
   api("us.ihmc:ihmc-footstep-planning:source")
   api("us.ihmc:simulation-construction-set-tools:source")
}

behaviorCleanRoomDependencies {
   compile ihmc.sourceSetProject("main")
   api("us.ihmc:ihmc-messager-kryo:0.1.3")
}

behaviorFxUiDependencies {
   api(ihmc.sourceSetProject("main"))
   api(ihmc.sourceSetProject("behavior-clean-room"))
   api("us.ihmc:ihmc-javafx-toolkit:0.14.1") {
      exclude(group = "org.slf4j", module = "slf4j-simple")
   }
   api("us.ihmc:ihmc-robot-models-visualizers:source")
   api("us.ihmc:ihmc-path-planning-visualizers:source")
   api("us.ihmc:robot-environment-awareness-visualizers:source")
}

testDependencies {
   compile ihmc.sourceSetProject("behavior-clean-room")
   compile ihmc.sourceSetProject("behavior-fx-ui")
   api("us.ihmc:ihmc-commons-testing:0.26.6")
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
   api("us.ihmc:ihmc-footstep-planning-test:source")
   api("us.ihmc:ihmc-humanoid-robotics-test:source")
   api("us.ihmc:simulation-construction-set-tools-test:source")
}
