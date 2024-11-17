if [[ ! -d build ]]; then
    echo "good"
else
    rm -rf build
fi
mkdir -p build
cd build
cmake ..
make -j

mkdir config
mkdir assets
mkdir figures
mkdir render
mkdir .cache
cp -r ../config/* config/
cp -r ../assets/* assets/