#!/bin/bash
set -e # Exit with nonzero exit code if anything fails
cd out

git rm -rf report/css/* || exit 0
git rm -rf report/data/* || exit 0
git rm -rf report/images/* || exit 0
git rm -rf report/js/* || exit 0
git rm -rf report/cocosReport.html || exit 0
git rm -rf report/grammarReport.html || exit 0
git rm -rf report/testReport.html || exit 0
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
