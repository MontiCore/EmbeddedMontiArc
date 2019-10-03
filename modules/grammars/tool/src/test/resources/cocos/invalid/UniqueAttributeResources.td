package invalid;

resources UniqueAttributeResources {
    attribute path = "some/awesome/path";
    attribute path = "some/awesome/path";

    resource SOME_RESOURCE {
        attribute path = "some/other/awesome/path";
        attribute path = "some/awesome/path";
    }
}
