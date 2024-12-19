if [[ ! -d build ]]; then
    mkdir build
fi
cd build
cmake ..
make -j

if [[ -d .cache ]]; then
    echo "good"
else
    rm -rf .cache
fi
mkdir .cache