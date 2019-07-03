package tagging;
conforms to dltag.DataPathTagSchema;

tags Alexnet {
tag Alexnet with DataPath = {path = src/test/resources/models, type = LMDB};
tag AlexnetInvalid with DataPath = {path = test/resources/models, type = random};
tag AlexnetInvalidType with DataPath = {path = src/test/resources/models, type = LMBD};
tag Parent.a1 with DataPath = {path = instanceA1, type = random};
}



