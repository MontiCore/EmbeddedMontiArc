/* (c) https://github.com/MontiCore/monticore */
package middleware.someip;
conforms to middleware.someip.SomeIPToEmamTagSchema;

tags OptionalMsgField{
tag optionalMsgField.in1 with SomeIPConnection = {topic = (name1, package/topic1)};
tag optionalMsgField.out1 with SomeIPConnection = {topic = (name1, package/topic1) , msgField = msgField1};
}
