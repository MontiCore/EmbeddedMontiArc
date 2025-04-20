MNISTCalculator
=================
42-automl: Problems with
- glounonnx: Missing skimage package 
- testemadlmavenplugin: Faulty settings, pom.xml -> works with master ones but misses same package

master:
- Secrets saved in commits -> no push possible (to public repo) need to be manually removed

git filter-repo --strip-blobs-with-ids <(echo "09492eb9dea3e3fcd7d8dfdd144c8aba7a7227a8 4c58767461e33c09ade2682c4648627a23c90abe 09492eb9dea3e3fcd7d8dfdd144c8aba7a7227a8 3e9f47945aa1c22653ff06aa8e59316367188677 66f53d40367bb1c110e54face5d7eb488e90cb0c")
clean secrets for public repo

16-create-example ...
-  base_mxnet Problemwith docker deployment

ma-antovski
- seems to work (even tough it fails)

modular_testing
- seems to work (even tough it fails)

ek
- seems to work (even tough it fails)

mnist_experiment
-  EMADL MavenPlugin: Mavn error dataset missing
-  Gluon Onnx: skimage missing


ma_fitzke
- works fine

EMADL2CPP
==============
Remove: 048427004c422c9fa10c5701b5a0d322758e17b7, 06c707b382c928cb72d23b41608e4d4d0196310b
Redo docker push login
Reod docker push url

113....
- buildPytorch: Dockerfile seems busted
- Otherwise seems to work

Problem with skipping as then downstream is not triggered
Solution idea: Add a stage job that decides whether the current stage is deemend successfull (all jobs either skipped or success) and if yes triggers next stage. All jobs in next stage then have need for this job