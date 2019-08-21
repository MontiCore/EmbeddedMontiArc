package valid;

language UniqueOption {
    declare template someTemplate for "SomePath" {
        attribute label = "Some Label";

        option someOption:string {
            prop label = "Some Option";
        }
    }
}