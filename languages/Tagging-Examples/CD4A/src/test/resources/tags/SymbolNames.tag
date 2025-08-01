/* (c) https://github.com/MontiCore/monticore */
package cd4a; //This needs to be the package of the cd model

conforms to tagschema.CD4ATags;  // not necessary to work, only needed for the TaggingGenerator and documentation

tags CD4ATags for MyClassDiagramm {
	tag MyClass with ClassTag = "A String tagged to the class MyClass"; // this will tag cd4a.MyClassDiagramm.MyClass
	tag MyClass.myVar with VariableTag = "A String tagged to the variable myVar"; // this will tag cd4a.MyClassDiagramm.MyClass.myVar
}
