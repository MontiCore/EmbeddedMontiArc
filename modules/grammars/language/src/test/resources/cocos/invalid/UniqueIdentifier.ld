package invalid;

import invalid.UniqueIdentifierParent;

language UniqueIdentifier extends UniqueIdentifierParent {
    declare template undeclareExisting for "SomePath" {
        attribute label = "Some Label";

        option someOption:string {
            prop label = "Some Option";
        }
    }
}
