const fs = require('fs');
const path = require('path');

console.log("Generating syntax file from keywords file");
let argv = checkArgs();

let packageJson = strictParse("package.json");
backup();
let parsedOptions = parseOptionFiles(argv[2]);

generateGrammarFiles(parsedOptions);
substituteGrammarsArray(packageJson, parsedOptions);
substituteLanguagesArray(packageJson, parsedOptions);
substituteActivationElements(packageJson, parsedOptions);

fs.writeFileSync(path.join(process.cwd(), "package.json"), JSON.stringify(packageJson, null, 2));



function backup() {
  console.log("Creating backup of package.json named package.old.json");
  fs.copyFileSync(path.join(process.cwd(), "package.json"), path.join(process.cwd(), "package.old.json"));
}

function checkArgs() {
  let argv = process.argv;
  if (argv.length != 3) {
    console.log("Expecting 1 Argument: path of global settings file " + (argv.length - 2));
    process.exit(1);
  }

  return argv;
}

function strictParse(fileName){
  if(!fs.existsSync(fileName)){
    console.log("Can not find " + fileName + ". Are you in the correct working directory?");
    process.exit(1);
  }

  return JSON.parse(fs.readFileSync(fileName));
}

function parseOptionFiles(globalSettingsPath) {
  let globalSettings = strictParse(globalSettingsPath);
  delete packageJson.contributes.languages;
  let parsedOptions = [];
  for (let a of globalSettings.clientOptions) {
    let tmpOptions = strictParse(a);
    parsedOptions.push(tmpOptions);
  }
  return parsedOptions;
}

function generateGrammarFiles(langOptions) {
  console.log("Generating grammar files!");

  for (let opt of langOptions) {
    const grammarFileName = opt.languageName + ".gen.tmGrammar.json";
    const keywordFileName = path.join(process.cwd(), opt.keywordsFile);
    const scopeName = getScopeName(opt);
    let res = generate(keywordFileName, scopeName);
    const resPath = path.join(process.cwd(), "syntax", "gen", grammarFileName);
    fs.writeFileSync(resPath, res, { encoding: "UTF-8" });
    console.log(opt.keywordsFile + "\t->\t" + "syntax/gen" + grammarFileName);
  }
}

function getScopeName(arg){
  return "keywords." + arg.languageName;
}

function substituteGrammarsArray(packageData, langOptions) {
  console.log("package.json - substituting contributes.grammars!");
  let grammars = [];
  let injectTo = [];

  for (let opt of langOptions) {
    const grammarFileName = opt.languageName + ".gen.tmGrammar.json";
    const scopeName = getScopeName(opt);
    let gram = {
      "language": opt.languageName,
      "scopeName": scopeName,
      "path": "syntax/gen/" + grammarFileName
    };

    injectTo.push(scopeName);
    grammars.push(gram);
  }

  grammars.push({
    "injectTo": injectTo,
    "scopeName": "monticore.comment",
    "path": "./syntax/comment.tmGrammar.json"
  });

  packageData.contributes.grammars = grammars;
}

function substituteLanguagesArray(packageData, langOptions) {
  console.log("package.json - substituting contributes.languages!");
  let languages = [];

  for (let opt of langOptions) {
    let lang = {
      "id": opt.languageName,
      "extensions": [
        opt.fileExtension
      ]
    };
    languages.push(lang);
  }

  packageData.contributes.languages = languages;
}

function substituteActivationElements(packageData, langOptions){
  console.log("package.json - substituting activationEvents!");
  let activationEvents = [];

  for (let opt of langOptions){
    activationEvents.push("onLanguage:" + opt.languageName);
  }

  packageData.activationEvents = activationEvents;
}


function generate(fileName, scopeName) {
  if (!fs.existsSync(fileName)) {
    console.log("Can not find " + fileName);
    process.exit(1);
  }
  let fileContent = fs.readFileSync(fileName, "utf8");
  let escapedFileContent = fileContent.replace(/[\/\\^$*+?.()|[\]{}]/g, '\\$&');
  let match = escapedFileContent
    .replace(/^/g, '\\\\b')
    .replace(/%/g, '\\\\b')
    .replace(/\r\n/g, "\\\\b|\\\\b")
    .replace(/\n/g, "\\\\b|\\\\b");
  if (!match.endsWith("\\n")) {
    match = match + "\\\\b";
  }
  let template = `
{
    "scopeName": "${scopeName}",
    "patterns": [{ "include": "#expression" }],
    "repository": {
      "expression": {
        "patterns": [{ "include": "#keyword" }]
      },
      "keyword": {
        "match": "${match}",
        "name": "keyword.other"
      }
    }
  }
`;
  return template;
}