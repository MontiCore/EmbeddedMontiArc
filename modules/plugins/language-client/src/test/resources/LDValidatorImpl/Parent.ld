package LDValidatorImpl;

language Parent {
    declare template parentTemplate for "SomePath" {
        attribute label = "Some Label";

        option someOption:string {
            prop label = "Some Option";
        }
    }
}