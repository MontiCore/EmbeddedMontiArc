/* (c) https://github.com/MontiCore/monticore */
// package de.serwth.emadl;

/*
* Schema definition for loss.
*/
schema Loss {

    loss {
        values:
            l1,
            l2,
            huber,
            cross_entropy,
            softmax_cross_entropy;

        define huber {
            rho: Q
        }

        define cross_entropy {
            sparse_label: B
            loss_axis: Z
            batch_axis: Z
        }

        define softmax_cross_entropy {
            sparse_label: B
            loss_axis: Z
            batch_axis: Z
            from_logits: B
        }
    }
}