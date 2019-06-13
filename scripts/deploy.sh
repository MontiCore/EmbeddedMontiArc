#!/bin/bash
#!/bin/sh
set -e # Exit with nonzero exit code if anything fails
SOURCE_BRANCH="master"
TARGET_BRANCH="report"

# Save some useful information
REPO=`git config remote.origin.url`
SSH_REPO=${REPO/https:\/\/github.com\//git@github.com:}
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
git rm -rf report/data/* || exit 0
mv ../report/data* report/

git add report/.
git commit -m "Deploy reports to report branch: ${SHA}"

git push $SSH_REPO $TARGET_BRANCH
cd ..
