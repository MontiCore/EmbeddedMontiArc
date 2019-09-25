/*
 * (c) https://github.com/MontiCore/monticore
 */
package valid;

import valid.UndeclareExistingParent;

language UndeclareExisting extends UndeclareExistingParent {
    declare template undeclareExisting for "SomePath" {
        attribute label = "Some Label";

        option someOption:string {
            prop label = "Some Option";
        }
    }

    undeclare template undeclareExistingParent;
}
