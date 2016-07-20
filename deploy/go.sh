DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
CURRENT_DIR=`pwd`
cd $DIR
cd ../pca10028/s130/armgcc && make clean
cd $DIR
cd ../pca10028/s130/armgcc && make
cd -
./loadbin.sh

cd $CURRENT_DIR
