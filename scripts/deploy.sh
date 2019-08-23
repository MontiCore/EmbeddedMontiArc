#!/bin/bash
# (c) https://github.com/MontiCore/monticore  
#!/bin/sh
set -e # Exit with nonzero exit code if anything fails
SOURCE_BRANCH="master"
TARGET_BRANCH="report"

# Save some useful information
REPO='https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/utilities/reporting'
PUSH_REPO=https://$GIT_USERNAME:$DEPLOY_KEY@git.rwth-aachen.de/monticore/EmbeddedMontiArc/utilities/reporting
SHA=`git rev-parse --verify HEAD`

if [ -d "out" ]
then
  rm -rf out
fi
git clone $REPO out
cd out

git checkout --track origin/$TARGET_BRANCH
git config user.name "GitLab CI"
git config user.email "malte.heithoff@rwth-aachen.de"

# deploy reports
git rm -rf public/report/data/* || exit 0
mv ../report/data public/report/
mv ../data/data* data/

git add public/report/.
git commit -m "Deploy reports to report branch: ${SHA}"

git push $PUSH_REPO $TARGET_BRANCH
cd ..
