package lab;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags LabTags {
    //syntax: tag $port with RosConnection = {topic=($topicName,$topicType), msgField=$fieldOfTopicType};

    tag system.alex.out1 with RosConnection = {topic=(/combine_in1,std_msgs/Int32), msgField=data};
    tag system.dinhAn.out1 with RosConnection = {topic=(/combine_in2,std_msgs/Int32), msgField=data};
    tag system.philipp.out1 with RosConnection = {topic=(/combine_in3,std_msgs/Int32), msgField=data};
    tag system.michael.out1 with RosConnection = {topic=(/combine_in4,std_msgs/Int32), msgField=data};

    tag system.combine.in1 with RosConnection = {topic=(/combine_in1,std_msgs/Int32), msgField=data};
    tag system.combine.in2 with RosConnection ={topic=(/combine_in2,std_msgs/Int32), msgField=data};
    tag system.combine.in3 with RosConnection = {topic=(/combine_in3,std_msgs/Int32), msgField=data};
    tag system.combine.in4 with RosConnection = {topic=(/combine_in4,std_msgs/Int32), msgField=data};

    tag system.alex.loop  with RosConnection = {topic=(/combine_out,std_msgs/Int32), msgField=data};
    tag system.dinhAn.loop with RosConnection = {topic=(/combine_out,std_msgs/Int32), msgField=data};
    tag system.philipp.loop with RosConnection = {topic=(/combine_out,std_msgs/Int32), msgField=data};
    tag system.michael.loop with RosConnection = {topic=(/combine_out,std_msgs/Int32), msgField=data};

    tag system.combine.out1 with RosConnection = {topic=(/combine_out,std_msgs/Int32), msgField=data};
}