/* (c) https://github.com/MontiCore/monticore */
package tests.structs;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags MatrixTypes{
tag matrixTypesComp.in1 with RosConnection = {topic = (/name1,std_msgs/Float64MultiArray)};
tag matrixTypesComp.in2 with RosConnection = {topic = (/name2,std_msgs/ByteMultiArray)};
tag matrixTypesComp.out1 with RosConnection = {topic = (/name3,std_msgs/ByteMultiArray)};
tag matrixTypesComp.out2 with RosConnection = {topic = (/name4,std_msgs/Int32MultiArray)};
}
