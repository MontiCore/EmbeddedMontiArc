/* (c) https://github.com/MontiCore/monticore */
package tests.structs;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags Structs{
    tag basicStructComp.posIn with RosConnection = {topic = (name1, struct_msgs/TestsStructsPosition)};
    tag nestedStructComp.posWithDtIn with RosConnection = {topic = (name2, struct_msgs/msg/TestsStructsPositionWithDeltaTime)};
    tag arrayStructComp.trajectoryIn with RosConnection = {topic = (name3, package/type3)};
}
