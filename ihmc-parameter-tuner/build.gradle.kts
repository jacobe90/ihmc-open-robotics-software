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
   api("net.sf.trove4j:trove4j:3.0.3")
   api("org.apache.commons:commons-lang3:3.8.1")
   api("org.apache.commons:commons-math3:3.3")
   api("us.ihmc:ihmc-commons:0.26.6")
   api("us.ihmc:ihmc-yovariables:0.4.0")
   api("us.ihmc:ihmc-javafx-toolkit:0.14.1") {
      exclude(group = "org.slf4j", module = "slf4j-simple")
   }
   api("us.ihmc:ihmc-robot-data-logger:0.15.0")
   api("us.ihmc:ihmc-robotics-toolkit:source")
}

testDependencies {
    api("us.ihmc:ihmc-robotics-toolkit-test:source")
}
