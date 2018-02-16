package tests.msg;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags Structs{
tag basicStructComp.in1 with RosConnection = {topic = (topic1,msgs/tests_structs_BasicStruct)};
tag basicStructComp.out1 with RosConnection = {topic = (topic2,msgs/tests_structs_BasicStruct)};

tag nestedStructComp.inNested with RosConnection = {topic = (topic3,msgs/tests_structs_NestedStruct)};
tag nestedStructComp.outNested with RosConnection = {topic = (topic4,msgs/tests_structs_NestedStruct)};

tag multiNestedStructComp.inMultiNested with RosConnection = {topic = (topic5,msgs/tests_structs_MultiNestedStruct)};
tag multiNestedStructComp.outMultiNested with RosConnection = {topic = (topic6,msgs/tests_structs_MultiNestedStruct)};
}