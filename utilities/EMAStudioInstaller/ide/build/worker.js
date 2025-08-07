// @ts-check
require("es6-promise/auto");
require("reflect-metadata");
const { Container } = require("inversify");
const { WorkerApplication } = require("@elysium/core/lib/worker/worker-application");
const { messagingWorkerModule } = require("@elysium/core/lib/worker/messaging/messaging-worker-module");

const container = new Container();

container.load(messagingWorkerModule);

function load(raw) {
    return Promise.resolve(raw.default).then(
        module => container.load(module)
    );
}

function start() {
    const application = container.get(WorkerApplication);
    application.start();
}

module.exports = Promise.resolve()
    .then(function () { return Promise.resolve(require("@elysium/cd/lib/worker/cd-worker-module")).then(load) })
    .then(function () { return Promise.resolve(require("@elysium/core/lib/worker/worker-application-module")).then(load) })
    .then(function () { return Promise.resolve(require("@elysium/embeddedmontiarc/lib/worker/embeddedmontiarc-worker-module")).then(load) })
    .then(function () { return Promise.resolve(require("@elysium/embeddedmontiarcmath/lib/worker/embeddedmontiarcmath-worker-module")).then(load) })
    .then(function () { return Promise.resolve(require("@elysium/embeddedmontiview/lib/worker/embeddedmontiview-worker-module")).then(load) })
    .then(function () { return Promise.resolve(require("@elysium/languages/lib/worker/languages-worker-module")).then(load) })
    .then(function () { return Promise.resolve(require("@elysium/macocoviz/lib/worker/macocoviz-worker-module")).then(load) })
    .then(function () { return Promise.resolve(require("@elysium/monticore/lib/worker/monticore-worker-module")).then(load) })
    .then(function () { return Promise.resolve(require("@elysium/montimath/lib/worker/montimath-worker-module")).then(load) })
    .then(function () { return Promise.resolve(require("@elysium/ocl/lib/worker/ocl-worker-module")).then(load) })
    .then(function () { return Promise.resolve(require("@elysium/streamunits/lib/worker/streamunits-worker-module")).then(load) })
    .then(start).catch(reason => {
        console.error("Failed to start the worker.");
        if (reason) {
            console.error(reason);
        }
    });