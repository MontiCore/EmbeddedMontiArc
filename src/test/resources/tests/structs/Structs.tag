package tests.structs;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags Structs{
tag basicStructComp.posIn with RosConnection = {topic = (name1,package/type1), msgField = field1};
tag nestedStructComp.posWithDtIn with RosConnection = {topic = (name2,package/type2), msgField = field2};
tag arrayStructComp.trajectoryIn with RosConnection = {topic = (name3,package/type3), msgField = field3};
}