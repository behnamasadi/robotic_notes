




/home/behnam/usr/bin/cmake ./thirdparty/gtsam -DGTSAM_BUILD_PYTHON=1 -B build_gtsam
/home/behnam/usr/bin/cmake --build build_gtsam --config RelWithDebInfo -j18



cmake -DPYTHON_LIBRARY=/home/behnam/anaconda3/envs/NeRF-SLAM/lib/libpython3.8.so \
-DPYTHON_INCLUDE_DIR=/home/behnam/anaconda3/envs/NeRF-SLAM/include/python3.8 \
-DPYTHON_EXECUTABLE=/home/behnam/anaconda3/envs/NeRF-SLAM/bin/python3



/home/behnam/usr/bin/cmake ./thirdparty/gtsam  -DGTSAM_PYTHON_VERSION=3.8.10 -DGTSAM_BUILD_PYTHON=1 -DPython3_LIBRARY=/home/behnam/anaconda3/envs/NeRF-SLAM/lib/libpython3.8.so \
-DPython3_INCLUDE_DIR=/home/behnam/anaconda3/envs/NeRF-SLAM/include/python3.8 \
-DPython3_EXECUTABLE=/home/behnam/anaconda3/envs/NeRF-SLAM/bin/python3 -B build_gtsam


cmake -DPython3_EXECUTABLE=/usr/local/bin/python3.8 
DPython3_FIND_STRATEGY=VERSION

/home/behnam/anaconda3/envs/NeRF-SLAM/lib/libpython3.8.so
/home/behnam/anaconda3/envs/NeRF-SLAM/include/python3.8
/home/behnam/anaconda3/envs/NeRF-SLAM/bin/python3




-DPython3_FIND_STRATEGY=VERSION









git clone https://github.com/gflags/gflags
cd gflags
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX:PATH=~/usr 
cmake --build build -j8
cmake --install build 



git clone https://github.com/google/glog
cd glog
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX:PATH=~/usr 
cmake --build build -j8
cmake --install build 


pip list -v


python3 -m pip show pip
