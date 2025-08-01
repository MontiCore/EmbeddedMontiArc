/* (c) https://github.com/MontiCore/monticore */
package arrays;
conforms to nfp.TraceabilityTagSchema;

tags Arrays{
//Subcomponent array without range
tag instanceArrayComp.simpleSubcomps[1] with Traceable;
tag instanceArrayComp.simpleSubcomps[3] with Traceable;

//Nested Subcomponent array without range
tag instanceArrayComp.nestedSubcomps[5].simpleSubcomps[4] with Traceable;

//Range syntax with step
//expected: 4,7
tag instanceArrayComp.simpleSubcomps[4:3:8] with Traceable;

//Range syntax without step
//expected: 20,21,...,30
tag instanceArrayComp.simpleSubcomps[20:30] with Traceable;

//Two ranges in one scope
//Expected: (1,1),(1,4),(3,1),(3,4)
tag instanceArrayComp.nestedSubcomps[1:2:3].simpleSubcomps[1:3:4] with Traceable;

}
