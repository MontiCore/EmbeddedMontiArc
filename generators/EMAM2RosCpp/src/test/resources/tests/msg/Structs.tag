/* (c) https://github.com/MontiCore/monticore */
package tests.msg;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags Structs{
tag basicStructComp.in1 with RosConnection = {topic = (topic1,struct_msgs/tests_structs_BasicStruct)};
tag basicStructComp.out1 with RosConnection = {topic = (topic2,struct_msgs/tests_structs_BasicStruct)};

tag nestedStructComp.inNested with RosConnection = {topic = (topic3,struct_msgs/tests_structs_NestedStruct)};
tag nestedStructComp.outNested with RosConnection = {topic = (topic4,struct_msgs/tests_structs_NestedStruct)};

tag multiNestedStructComp.inMultiNested with RosConnection = {topic = (topic5,struct_msgs/tests_structs_MultiNestedStruct)};
tag multiNestedStructComp.outMultiNested with RosConnection = {topic = (topic6,struct_msgs/tests_structs_MultiNestedStruct)};

tag basicTypesComp.inQ with RosConnection = {topic = (topic7,std_msgs/Float64)};
tag basicTypesComp.inZ with RosConnection = {topic = (topic8,std_msgs/Int32)};
tag basicTypesComp.inB with RosConnection = {topic = (topic9,std_msgs/Bool)};
tag basicTypesComp.outQ with RosConnection = {topic = (topic7,std_msgs/Float64)};
tag basicTypesComp.outZ with RosConnection = {topic = (topic8,std_msgs/Int32)};
tag basicTypesComp.outB with RosConnection = {topic = (topic9,std_msgs/Bool)};
}
