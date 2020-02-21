if [ -z "$2" ]; then
  PYTHONPATH=$(which python3)
else
  PYTHONPATH = $2;
fi
echo "Checking for opencv"
if $PYTHONPATH -c "import cv2" ; then
  echo "opencv installed"
else
  echo "opencv required"
  exit 1
fi

echo "Checking for h5py"
if $PYTHONPATH -c "import h5py" ; then
  echo "h5py installed"
else
  echo "h5py required"
  exit 1
fi

FILE=$1

if [[ $FILE != "cityscapes" &&  $FILE != "night2day" &&  $FILE != "edges2handbags" && $FILE != "edges2shoes" && $FILE != "facades" && $FILE != "maps" ]]; then
  echo "Available datasets are cityscapes, night2day, edges2handbags, edges2shoes, facades, maps"
  exit 1
fi

echo "Specified [$FILE]"

if [[$FILE == "cityscapes"]];  then
declare -i i=3
declare -i o=3
fi

if [[$FILE == "night2day"]];  then
declare -i i=3
declare -i o=3
fi

if [[$FILE == "edges2handbags"]];  then
declare -i i=1
declare -i o=3
fi

if [[$FILE == "edges2shoes"]];  then
declare -i i=1
declare -i o=3
fi

if [ "facades" == $FILE ];  then
declare -i i=3
declare -i o=3
fi

if [[$FILE == "maps"]];  then
declare -i i=3
declare -i o=3
fi

# URL=http://efrosgans.eecs.berkeley.edu/pix2pix/datasets/$FILE.tar.gz
# TAR_FILE=./datasets/$FILE.tar.gz
TARGET_DIR=./datasets/$FILE/
# wget -N $URL -O $TAR_FILE
# mkdir -p $TARGET_DIR
# tar -zxf $TAR_FILE -C ./datasets/
# rm $TAR_FILE

$PYTHONPATH dataset2h5.py -p $TARGET_DIR -i $i -o $o -t n

# rm -rf ./$TARGET_DIR/test
# rm -rf ./$TARGET_DIR/train
# rm -rf ./$TARGET_DIR/val