schema Pipeline {

/*
  implicitly equivalent to following declarations. Schema should therefore avoid redeclaration as this would lead to validation error
    data_access : component
    training : component
    evaluation : component
*/

reference-model = referencemodels.Training_Pipeline

/*  possible pipeline steps
    step names should correpsond to those of components in the corresponding reference model(s) to make symbol linking easier
*/

Step {
  values:
	Data_Access,
	Training;

//determines tep order
 id:N!

 define Training{
    implementation = SupervisedTrainer: component!
    // should correpsond to those of components in the corresponding reference model(s)
    network: component!
 }

 define Data_Access{
    implementation = HDF5DataAccess:component!
    datasource: component!
 }

}

// Pipeline steps
   steps {
    values: useless_keyword;

    train_step: Step!
    data_access_step: Step!
   }
}