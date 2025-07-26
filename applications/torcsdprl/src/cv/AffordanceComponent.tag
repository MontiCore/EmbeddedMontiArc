package cv;

conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags AffordanceComponent {
    tag affordanceComponent.vision with RosConnection = {topic=(torcs/vision, std_msgs/UInt8MultiArray)};
    tag affordanceComponent.affordance with RosConnection = {topic=(torcs/affordance, std_msgs/Float32MultiArray)};
}


