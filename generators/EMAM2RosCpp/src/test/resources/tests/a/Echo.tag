/* (c) https://github.com/MontiCore/monticore */
package tests.a;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags Echo {
tag compA.rosIn with RosConnection = {topic = (/clock, rosgraph_msgs/Clock), msgField = clock.toSec()};
tag compA.rosOut with RosConnection = {topic = (/echo, automated_driving_msgs/StampedFloat64), msgField = data};
}
