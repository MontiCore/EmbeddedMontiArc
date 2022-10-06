/* (c) https://github.com/MontiCore/monticore */

schema GNN extends Supervised {
    train_mask: Z*
    test_mask: Z*
    multi_graph: B
}
