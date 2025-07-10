/* (c) https://github.com/MontiCore/monticore */
package tagging;
conforms to dltag.DataPathTagSchema;

tags AlexNet {
tag Alexnet with DataPath = {path = data, type = random};
tag Parent.a1 with DataPath = {path = src/test/models/, type = LMDB};
tag Parent.a2 with DataPath = {path = lisjef, type = r34};
}
