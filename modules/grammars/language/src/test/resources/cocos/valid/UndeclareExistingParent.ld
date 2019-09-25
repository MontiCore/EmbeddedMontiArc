/*
 * (c) https://github.com/MontiCore/monticore
 */
package valid;

language UndeclareExistingParent {
    declare template undeclareExistingParent for "SomePath" {
        attribute label = "Some Label";

        option someOption:string {
            prop label = "Some Option";
        }
    }
}
