cwd=$(pwd)
tar -zxvf Ipopt-3.12.8.tar.gz

cd $cwd/Ipopt-3.12.8/ThirdParty/Blas
./get.Blas
cd ../Lapack
./get.Lapack
cd ../Mumps
./get.Mumps
cd ../Metis
./get.Metis

cd $cwd/Ipopt-3.12.8
mkdir build
cd build
../configure
make -j4
make install

cd $cwd/Ipopt-3.12.8/build
sudo cp -a include/* /usr/include/.
sudo cp -a lib/* /usr/lib/.

cd $cwd
rm -r Ipopt-3.12.8
