/* (c) https://github.com/MontiCore/monticore */
package middleware.someip;
conforms to middleware.someip.SomeIPToEmamTagSchema;

tags Echo {
tag basicParsing.someIPIn with SomeIPConnection = {serviceID = 1, instanceID = 2, eventgroupID = 3};
tag basicParsing.someIPOut with SomeIPConnection = {serviceID = 1, instanceID = 2, eventgroupID = 3};
tag basicParsing.emptyTagIn with SomeIPConnection;
}
