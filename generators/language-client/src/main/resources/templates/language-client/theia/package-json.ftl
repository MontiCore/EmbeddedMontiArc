{
  "name": "@embeddedmontiarc/language-${grammar}",
  "version": "${version}",
  "description": "Theia extension for the ${Grammar} Language.",
  "keywords": ["theia-extension", "EmbeddedMontiArc Language"],
  "author": "SE RWTH",
  "files": ["lib", "src", "data"],
  "dependencies": {
    "@theia/core": "latest",
    "@theia/languages": "latest",
    "@theia/monaco": "latest",
    "@theia/process": "latest"
  },
  "devDependencies": {
    "rimraf": "latest",
    "typescript": "latest"
  },
  "scripts": {
    "prepare": "yarn run clean && yarn run build",
    "clean": "rimraf lib",
    "build": "tsc",
    "watch": "tsc -w"
  },
  "theiaExtensions": [
    {
      "frontend": "lib/browser/embeddedmontiarc-frontend-module"
    }
  ]
}
