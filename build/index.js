// @ts-check
require('es6-promise/auto');
require('reflect-metadata');
const { Container } = require('inversify');
const { FrontendApplication } = require('@theia/core/lib/browser');
const { frontendApplicationModule } = require('@theia/core/lib/browser/frontend-application-module');
const { messagingFrontendModule } = require('@theia/core/lib/browser/messaging/messaging-frontend-module');
const { loggerFrontendModule } = require('@theia/core/lib/browser/logger-frontend-module');

const container = new Container();
container.load(frontendApplicationModule);
container.load(messagingFrontendModule);
container.load(loggerFrontendModule);

function load(raw) {
    return Promise.resolve(raw.default).then(module =>
        container.load(module)
    )
}

function start() {
    const application = container.get(FrontendApplication);
    application.start();
}

module.exports = Promise.resolve()
    .then(function () { return Promise.resolve(require('@theia/core/lib/browser/menu/browser-menu-module')).then(load) })
    .then(function () { return Promise.resolve(require('@theia/core/lib/browser/window/browser-window-module')).then(load) })
    .then(function () { return Promise.resolve(require('@theia/output/lib/browser/output-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@theia/languages/lib/browser/languages-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@theia/editor/lib/browser/editor-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@theia/filesystem/lib/browser/filesystem-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@theia/workspace/lib/browser/workspace-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@theia/navigator/lib/browser/navigator-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@theia/markers/lib/browser/problem/problem-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@theia/outline-view/lib/browser/outline-view-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@theia/monaco/lib/browser/monaco-browser-module')).then(load) })
    .then(function () { return Promise.resolve(require('@theia/callhierarchy/lib/browser/callhierarchy-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@theia/userstorage/lib/browser/user-storage-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@theia/preferences/lib/browser/preference-frontend-module')).then(load) })
    //.then(function () { return Promise.resolve(require('@theia/cpp/lib/browser/cpp-frontend-module')).then(load) })
    //.then(function () { return Promise.resolve(require('@theia/extension-manager/lib/browser/extension-frontend-module')).then(load) })
    //.then(function () { return Promise.resolve(require('@theia/file-search/lib/browser/file-search-frontend-module')).then(load) })
    //.then(function () { return Promise.resolve(require('@theia/git/lib/browser/git-frontend-module')).then(load) })
    //.then(function () { return Promise.resolve(require('@theia/java/lib/browser/java-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@theia/keymaps/lib/browser/keymaps-frontend-module')).then(load) })
    //.then(function () { return Promise.resolve(require('@theia/merge-conflicts/lib/browser/merge-conflicts-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@theia/messages/lib/browser/messages-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@theia/mini-browser/lib/browser/mini-browser-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@theia/preview/lib/browser/preview-frontend-module')).then(load) })
    //.then(function () { return Promise.resolve(require('@theia/python/lib/browser/python-frontend-module')).then(load) })
    //.then(function () { return Promise.resolve(require('@theia/search-in-workspace/lib/browser/search-in-workspace-frontend-module')).then(load) })
    //.then(function () { return Promise.resolve(require('@theia/terminal/lib/browser/terminal-frontend-module')).then(load) })
    //.then(function () { return Promise.resolve(require('@theia/task/lib/browser/task-frontend-module')).then(load) })
    //.then(function () { return Promise.resolve(require('@theia/typescript/lib/browser/typescript-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@elysium/cd/lib/browser/cd-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@elysium/core/lib/browser/frontend-application-module')).then(load) })
    .then(function () { return Promise.resolve(require('@elysium/core/lib/browser/window/browser-window-module')).then(load) })
    .then(function () { return Promise.resolve(require('@elysium/core/lib/browser/history/browser-history-module')).then(load) })
    .then(function () { return Promise.resolve(require('@elysium/core/lib/browser/logger-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@elysium/core/lib/browser/messaging/messaging-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@elysium/dashboard/lib/browser/filesystem/filesystem-dashboard-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@elysium/dashboard/lib/browser/demos/demos-dashboard-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@elysium/downloader/lib/browser/downloader-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@elysium/embeddedmontiarc/lib/browser/embeddedmontiarc-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@elysium/embeddedmontiarcmath/lib/browser/embeddedmontiarcmath-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@elysium/embeddedmontiview/lib/browser/embeddedmontiview-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@elysium/filesystem/lib/browser/filesystem-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@elysium/languages/lib/browser/languages-frontend-module')).then(load) })
    //.then(function () { return Promise.resolve(require('@elysium/initial/lib/browser/initial-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@elysium/macocoviz/lib/browser/macocoviz-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@elysium/messages/lib/browser/messages-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@elysium/mini-browser/lib/browser/mini-browser-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@elysium/monaco/lib/browser/monaco-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@elysium/montimath/lib/browser/montimath-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@elysium/navigator/lib/browser/navigator-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@elysium/workspace/lib/browser/workspace-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@elysium/monticore/lib/browser/monticore-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@elysium/ocl/lib/browser/ocl-frontend-module')).then(load) })
    .then(function () { return Promise.resolve(require('@elysium/se-logo/lib/browser/se-logo-module')).then(load) })
    .then(function () { return Promise.resolve(require('@elysium/streamunits/lib/browser/streamunits-frontend-module')).then(load) })
    .then(start).catch(reason => {
        console.error('Failed to start the frontend application.');
        if (reason) {
            console.error(reason);
        }
    });