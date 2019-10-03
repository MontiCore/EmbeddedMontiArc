package invalid;

tool ResourceName {
    attribute path = "some/awesome/path";

    resource someResource {
        attribute path = "some/other/awesome/path";
    }
}
