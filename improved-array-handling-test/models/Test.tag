package test;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags Test{
	tag test.test_in with RosConnection = {topic = (/test_in, std_msgs/Float64MultiArray), msgField = data[3:8]};
	tag test.test_out with RosConnection = {topic = (/test_out, std_msgs/Float64MultiArray)};
}

