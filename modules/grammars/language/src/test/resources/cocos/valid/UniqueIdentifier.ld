package valid;

import valid.UniqueIdentifierParent;

language UniqueIdentifier extends UniqueIdentifierParent {
    declare template undeclareExisting2 for "SomePath" {
        attribute label = "Some Label";

        option someOption:string {
            prop label = "Some Option";
        }
    }
}
