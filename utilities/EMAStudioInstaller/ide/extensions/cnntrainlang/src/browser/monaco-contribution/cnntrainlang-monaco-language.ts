/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

/// <reference types="@theia/monaco/src/typings/monaco" />

import { DEFAULT_LANGUAGE_CONFIGURATION, DEFAULT_MONARCH_LANGUAGE } from "@elysium/languages/lib/browser";

export const configuration: monaco.languages.LanguageConfiguration = DEFAULT_LANGUAGE_CONFIGURATION;

export const monarchLanguage: monaco.languages.IMonarchLanguage = Object.assign({}, DEFAULT_MONARCH_LANGUAGE, {
    tokenPostfix: ".cnnt",
    keywords: [
        "configuration", "num_epoch", "batch_size", "load_checkpoint", "normalize", "optimizer", "context", "sgd",
        "adam", "rmsprop", "adagrad", "nag", "adadelta", "learning_rate_minimum", "learning_rate", "weight_decay",
        "learning_rate_decay", "learning_rate_policy", "rescale_grad", "clip_gradient", "step_size", "momentum",
        "beta1", "beta2", "epsilon", "gamma1", "gamma2", "centered", "clip_weights", "rho"
    ],
    operators: [
        "true", "false", "fixed", "step", "exp", "inv", "poly", "sigmoid", "cpu", "gpu"
    ]
});
