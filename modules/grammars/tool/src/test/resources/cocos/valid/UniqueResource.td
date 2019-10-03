package valid;

tool UniqueResource {
    attribute path = "some/awesome/path";

    resource SOME_RESOURCE {
        attribute path = "some/other/awesome/path";
    }
}
