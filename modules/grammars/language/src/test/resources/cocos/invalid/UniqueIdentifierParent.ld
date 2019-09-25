/*
 * (c) https://github.com/MontiCore/monticore
 */
package invalid;

language UniqueIdentifierParent {
    declare template undeclareExisting for "SomePath" {
        attribute label = "Some Label";

        option someOption:string {
            prop label = "Some Option";
        }
    }
}
