package invalid;

resources ValidAttributeResources {
    attribute path = "some/strange/path";

    resource SOME_RESOURCE {
         attribute label = "Some Other Label?";
     }
}