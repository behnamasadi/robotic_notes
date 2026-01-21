## CUDA settings

```
export CC=/usr/bin/gcc-9
export CXX=/usr/bin/g++-9
export CUDAHOSTCXX=/usr/bin/g++-9

```

find which version of cuda:

```
sudo update-alternatives --display cuda
sudo update-alternatives --config cuda
```

setting version of cuda

```
export PATH="/usr/local/cuda-11.8/bin:$PATH"
export LD_LIBRARY_PATH="/usr/local/cuda-11.8/lib64:$LD_LIBRARY_PATH"
```
for instance:

```
export PATH="/usr/local/<cuda-version>/bin:$PATH"
export LD_LIBRARY_PATH="/usr/local/<cuda-version>/lib64:$LD_LIBRARY_PATH"
```

## Building 


```
cmake -S . -B build -DCMAKE_INSTALL_PREFIX=~/usr
cmake --build build --config RelWithDebInfo -j
```


## colmap2nerf

```
python3 ./scripts/colmap2nerf.py --colmap_matcher exhaustive --run_colmap --aabb_scale 32 --images=/home/behnam/Pictures/south-building/images/_resized
```
