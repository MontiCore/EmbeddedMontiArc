package invalid;

resources UniqueResourceResources {
    resource SOME_RESOURCE {
        attribute path = "some/other/awesome/path";
    }

    resource SOME_RESOURCE {
        attribute path = "some/other/awesome/path";
    }
}
