if [ ! -d "$1" ]
then
  mkdir "$1"
fi

cd $1

for entry in $(curl "https://api.github.com/orgs/$1/repos?per_page=200" | grep -e 'git_url*' | cut -d \" -f 4)
do
  url=$entry
  rep=$(basename $entry)
  rep="${rep%.*}"
  if [ -d $rep ]
  then
    cd $rep && git pull && cd ..
  elif [ $rep != "external-dependencies" ]
  then
    git clone "https://github.com/$1/$rep"
  fi
done
cd ..
