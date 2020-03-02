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
   api("org.ejml:core:0.30")
   api("org.ejml:dense64:0.30")

   api("us.ihmc:euclid:0.12.2")
   api("us.ihmc:ihmc-yovariables:0.4.0")
   api("us.ihmc:ihmc-graphics-description:0.14.1")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ihmc-convex-optimization:0.13.0")
   api("us.ihmc:ihmc-path-planning:source")
   api("us.ihmc:ihmc-humanoid-robotics:source")
   api("us.ihmc:ihmc-common-walking-control-modules:source")
   api("us.ihmc:ihmc-path-planning-data-sets:source")
}

testDependencies {
   api(ihmc.sourceSetProject("visualizers"))

   api("us.ihmc:ihmc-path-planning-data-sets:source")
   api("us.ihmc:simulation-construction-set-test:0.14.0")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ihmc-path-planning-test:source")
   api("us.ihmc:simulation-construction-set-tools-test:source")
}

visualizersDependencies {
   api(ihmc.sourceSetProject("main"))

   api("us.ihmc:ihmc-javafx-toolkit:0.14.1") {
      exclude(group = "org.slf4j", module = "slf4j-simple")
   }
   api("us.ihmc:robot-environment-awareness-application:source")
   api("us.ihmc:ihmc-communication:source")
   api("us.ihmc:ihmc-path-planning-visualizers:source")
   api("us.ihmc:ihmc-robot-models:source")
   api("us.ihmc:ihmc-robot-models-visualizers:source")
   api("us.ihmc:ihmc-whole-body-controller:source")
}
