/* (c) https://github.com/MontiCore/monticore */

schema SchemaWithObjectTypeEntry{

   optimizer_type {
           values:
               sgd,
               adam,
               adamw;

           /*
           * Parameters common to all optimizer algorithms.
           */
           learning_rate: Q
           learning_rate_policy: enum {
               fixed, step, exp, inv, poly, sigmoid;
           }

           define sgd {
               momentum: Q
           }

           define adam {
               beta1: Q
               beta2: Q
           }
       }
   optimizer: optimizer_type
   optimizer2: optimizer_type


}