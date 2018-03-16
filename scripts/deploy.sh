#!/bin/bash
set -e # Exit with nonzero exit code if anything fails
SOURCE_BRANCH="master"
TARGET_BRANCH="gh-pages"

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

git checkout $TARGET_BRANCH || git checkout --orphan $TARGET_BRANCH
git config user.name "Travis CI"
git config user.email "$COMMIT_AUTHOR_EMAIL"

# deploy Zip
git rm *.zip

for var in "$@"
do
    mv ../$var $var
done

git add *.zip
git commit -m "Deploy new Zips: ${SHA}"

# deploy reports
git rm -rf report/css/* || exit 0
git rm -rf report/data/* || exit 0
git rm -rf report/images/* || exit 0
git rm -rf report/js/* || exit 0
git rm -rf report/*.html
if [ ! -d "report" ]
then
  mkdir report
fi
mkdir report/css
mkdir report/data
mkdir report/images
mkdir report/js
mv ../report/css/* report/css
mv ../report/data/* report/data
mv ../report/images/* report/images
mv ../report/js/* report/js
mv ../report/cocosReport.html report/cocosReport.html
mv ../report/grammarReport.html report/grammarReport.html
mv ../report/testReport.html report/testReport.html

git add report/.
git commit -m "Deploy Reports to GitHub Pages: ${SHA}"

git push $SSH_REPO $TARGET_BRANCH
cd ..
