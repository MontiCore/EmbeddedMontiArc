/* (c) https://github.com/MontiCore/monticore */
const Process = require("./process");
const Log = require("log4js");
const {EXECUTABLES, JARS} = require("./constants");

class EMAM2EMA {
    constructor() {
        this.logger = Log.getLogger("EMAM2EMA");
    }

    execute(root) {
        const arguments = ["-jar", JARS.EMAM2EMA, root];

        this.logger.info("Transforming EMAM Models to EMA Models...");
        return Process.spawn(EXECUTABLES.JAVA, arguments);
    }
}

module.exports = new EMAM2EMA();
