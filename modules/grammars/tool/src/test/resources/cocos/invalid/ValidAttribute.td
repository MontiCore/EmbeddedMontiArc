package invalid;

tool ValidAttribute {
    attribute label = "Some Label?";

    resource SOME_RESOURCE {
        attribute label = "Some Other Label?";
    }
}
