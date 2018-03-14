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
git rm -rf report/css/* || exit 0
git rm -rf report/data/* || exit 0
git rm -rf report/images/* || exit 0
git rm -rf report/js/* || exit 0
git rm -rf report/report.html || exit 0
git rm -rf report/reportEWT.html || exit 0
git rm -rf report/grammarReport.html || exit 0
if [ ! -d "report" ]
then
  mkdir report
fi
mkdir report/css
mv ../report/css/* report/css
mkdir report/data
mv ../report/data/* report/data
mkdir report/images
mv ../report/images/* report/images
mkdir report/js
mv ../report/js/* report/js
mv ../report/report.html report/report.html
mv ../report/reportEWT.html report/reportEWT.html
mv ../report/grammarReport.html report/grammarReport.html

git config user.name "Travis CI"
git config user.email "$COMMIT_AUTHOR_EMAIL"

git add report/.
git commit -m "Deploy to GitHub Pages: ${SHA}"

# Now that we're all set up, we can push.
git push $SSH_REPO $TARGET_BRANCH
cd ..
