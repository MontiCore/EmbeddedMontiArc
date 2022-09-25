schema DefinedNestedEntriesInReferenceModel {

/*
  implicitly equivalent to following declarations. Schema should therefore avoid redeclaration as this would lead to validation error
    data_access : component
    training : component
    evaluation : component
*/

reference-model : mlpipeline.SupervisedPipeline
}