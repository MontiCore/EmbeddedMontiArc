/* (c) https://github.com/MontiCore/monticore */
package middleware.someip;
conforms to middleware.someip.SomeIPToEmamTagSchema;

tags Echo {
tag basicParsing.someIPIn with SomeIPConnection = {topic = (/clock, someip/Clock), msgField = clock.toSec()};
tag basicParsing.someIPOut with SomeIPConnection = {topic = (/echo, automated_driving_msgs/StampedFloat64), msgField = data};
tag basicParsing.emptyTagIn with SomeIPConnection;
}
