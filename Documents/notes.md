MNISTCalculator
=================
42-automl: Problems with

- glounonnx: Missing skimage package
- testemadlmavenplugin: Faulty settings, pom.xml → works with master ones but misses same package

master:

- Secrets saved in commits → no push possible (to public repo) need to be manually removed

#git filter-repo --strip-blobs-with-ids <(echo "09492eb9dea3e3fcd7d8dfdd144c8aba7a7227a8
4c58767461e33c09ade2682c4648627a23c90abe 09492eb9dea3e3fcd7d8dfdd144c8aba7a7227a8
3e9f47945aa1c22653ff06aa8e59316367188677 66f53d40367bb1c110e54face5d7eb488e90cb0c")
#clean secrets for public repo

16-create-example ...

- base_mxnet Problem with docker deployment

ma-antovski

- seems to work (even tough it fails)

modular_testing

- seems to work (even tough it fails)

ek

- seems to work (even tough it fails)

mnist_experiment

- EMADL MavenPlugin: Mavn error dataset missing
- Gluon Onnx: skimage missing

ma_fitzke

- works fine

EMADL2CPP
==============
#Remove: 048427004c422c9fa10c5701b5a0d322758e17b7, 06c707b382c928cb72d23b41608e4d4d0196310b
Redo docker push login
Reod docker push url
Change token in artifact extractor CI to Private token
GIT jobs untouched
extractor/extract.py needs to be reworked to new repo adresses
buildJobs not compared, but work (Dockerfiles sometimes broken)

113....

- buildPytorch: Dockerfile seems busted
- Otherwise seems to work

AK_Hyperband

- seems to work (even tough it fails)

49...

- seems to work

62...

- seems to work

Akash Hiroshi

- seems to work

Baysian_Opt

- seems to work (even tough it fails)

GeneticAlgorithm

- seems to work

PSO

- seems to work

SA

- seems to work

add_numcites

- seems to work

ba_weber

- buildTensorflowOnnx: Dockerfile seems busted
- Otherwise hard to say Error in tests

conf_parser

- seems to work (even tough it fails) to second to last run

dev

- seems to work (even tough it fails)

docu_onboarding

- seems to work

ma-daxhammer

- seems to work

ma-lo

- PythonWrapper: Fails weirdly because maven vm crashes
- UnitTestLinux: One 9 more failed tests on github?

ma_akash_new

- UnitTestLinux: One 6 failed tests on github?

ma_mulhem

- UnitTestLinux: One more failed test on github?
- IntegrationMXNET: One test less?
- PythonWrapper: Fails weirdly because maven vm crashes

marvin

- seems to work

master

- seems to work

python37

- seems to work (even tough it fails)

ranges_handler

- seems to work

sac

- seems to work

ToDo
=================

- [x] Problem with skipping as then downstream is not triggered
  Solution idea: Add a stage job that decides whether the current stage is deemend successfull (all jobs either skipped
  or success) and if yes triggers next stage. All jobs in next stage then have need for this job


- [x] Pages deployment

- [x] Add on file change

- [x] **Can I run arbitraty pipelines in GitLab to compare results?**

- [x] Add function to change docker image in action if they are to be migrated

- [x] Add function that scans script line and if necessary implements changes
    - Change CI_Token to GITLAB_TOKEN
    - Change REGISTRY in docker login
    - Change password in docker login
    - Change docker tag to github
    - Change docker push to github

- [x] Add secrets from GitLab and variables to architecture.yaml, then manual input of values required there

- [x] Scan for secrets and Variables in Maven files

- [x] Verify LFS

- [x] Implement usage as CLI, implement adaptability in the script

- [x] Implement usage as library

- [ ] Implement retry mechanism for migration, so that no double commits

- [x] Maybe local secret scanning with 3rd party tool? Will not be implemented

- [x] Need organization acces to Monticore -> Create extra repos, as no folder structure as in GitLab: Only monorepo

- [x] Change tool to be independent of user Namespace, currently only works for David.Blum -> Change to repo owner
  variable in jobs -> remove config entry

- [x] Add pre-commit hook to check for file size -> if yes split? Add pull-hook to rebuild those large files -> omitted

- [x] Make sure user push protection is set to false -> Has to be done manually, no API access

- [x] Enable /disable Push protection in Repo

- [x] Push in multiple steps, so that 2GB limit is not hit

- [x] Implement multiple branches in monorepo

- [x] Change brnach condition to: github.ref_name == 'master'

- [x] Change filejob to als check except

- [x] Importer not suited because seperate runs for each script line -> separate shells mention in begining

- [x] Add fileChanges dependency for downstream jobs -> important for rules

- [x] Change trigger similar to: https://github.com/github/gh-actions-importer/blob/main/docs/gitlab/Trigger.md

- [x] Add reporting action

- [ ] Add second pages trigger

