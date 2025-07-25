/* (c) https://github.com/MontiCore/monticore */
const MakeTestCafe = require("testcafe");

const BROWSERS = process.env.HOST ? ["chrome"] : [
    "browserstack:chrome:Windows 10",
    "browserstack:firefox:Windows 10",
    "browserstack:edge:Windows 10"
];

const TESTS = [
    "modules/se.rwth.common.settings/test/settings.test.js",
    "modules/se.rwth.common.loader/test/loader.test.js",
    "modules/se.rwth.common.dialog/test/dialog.test.js",
    "modules/se.rwth.common.checker/test/checker.test.js",
    "modules/se.rwth.common.dashboard/test/dashboard.test.js"
];

let runner = null;
let testcafe = null;

MakeTestCafe("localhost", 8085).then(tc => {
    testcafe = tc;
    runner = testcafe.createRunner();

    return runner.src(TESTS).browsers(BROWSERS).run();
}).then(() => {
    testcafe.close();
}).catch((error) => {
    console.error(error);
});
