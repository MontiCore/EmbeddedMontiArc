/*
 * (c) https://github.com/MontiCore/monticore
 */
package valid;

language UniqueIdentifierParent {
    declare template undeclareExisting for "SomePath" {
        attribute label = "Some Label";

        option someOption:string {
            prop label = "Some Option";
        }
    }
}
