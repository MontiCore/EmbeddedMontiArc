# Procedure for schema2api
    * Current limitations
        * no  reference models
        * only primitive types in complex property definitions
        * no exceptions thrown for nonoccuring hyperparams
        * no type hints/ annotations
        * no imports -> composition
        * only one super schema supported
        * no possibility to adapt target python version
        * add TOP mechanism ?
        * need of smarter way to handle paddings
    * agenda
        * initialization logic for training configuration reference
        * all types must be handled: enums, schemas
        * unit tests to compare with expected results
        * functional example with one complete hierarchy