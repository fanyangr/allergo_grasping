mkdir -p build &&
cd build &&
cmake .. &&
make &&
make install &&
cp -rf bin/grasp_driver ../ &&
cd ..

