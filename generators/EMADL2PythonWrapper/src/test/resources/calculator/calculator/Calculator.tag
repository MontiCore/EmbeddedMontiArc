/* (c) https://github.com/MontiCore/monticore */
package calculator;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags Calculator {
    tag calculator.a with RosConnection = {topic=(/a, std_msgs/Float32MultiArray)};
    tag calculator.b with RosConnection = {topic=(/b, std_msgs/Float32MultiArray)};
    tag calculator.c with RosConnection = {topic=(/c, std_msgs/Int32)};
    tag calculator.result with RosConnection = {topic=(/commands, std_msgs/Float32)};
}
