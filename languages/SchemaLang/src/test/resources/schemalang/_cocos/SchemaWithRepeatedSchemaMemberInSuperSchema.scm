/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.schemalang;

schema SchemaWithRepeatedSchemaMemberInSuperSchema extends SuperSchema {
    num_epoch = 100000: N1
    batch_size = 64: N1
}