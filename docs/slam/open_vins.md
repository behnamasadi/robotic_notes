## OpenVINS Multi-Camera Extension



```
<!-- bag topics -->
<param name="topic_imu"      type="string" value="/imu0" />
<param name="topic_camera0"  type="string" value="/cam0/image_raw" />
<param name="topic_camera1"  type="string" value="/cam1/image_raw" />
<param name="topic_camera2"  type="string" value="/cam0/image_raw" />
<rosparam param="stereo_pairs">[0,1]</rosparam>

<!-- camera intrinsics -->
<rosparam param="cam0_wh">[752, 480]</rosparam>
<rosparam param="cam1_wh">[752, 480]</rosparam>
<rosparam param="cam2_wh">[752, 480]</rosparam>
<param name="cam0_is_fisheye" type="bool" value="false" />
<param name="cam1_is_fisheye" type="bool" value="false" />
<param name="cam2_is_fisheye" type="bool" value="false" />
<rosparam param="cam0_k">[458.654,457.296,367.215,248.375]</rosparam>
<rosparam param="cam0_d">[-0.28340811,0.07395907,0.00019359,1.76187114e-05]</rosparam>
<rosparam param="cam1_k">[457.587,456.134,379.999,255.238]</rosparam>
<rosparam param="cam1_d">[-0.28368365,0.07451284,-0.00010473,-3.55590700e-05]</rosparam>
<rosparam param="cam2_k">[458.654,457.296,367.215,248.375]</rosparam>
<rosparam param="cam2_d">[-0.28340811,0.07395907,0.00019359,1.76187114e-05]</rosparam>

<!-- camera extrinsics -->
<rosparam param="T_C0toI">
    [
    0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
    0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
    -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
    0.0, 0.0, 0.0, 1.0
    ]
</rosparam>
<rosparam param="T_C1toI">
    [
    0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
    0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
    -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
    0.0, 0.0, 0.0, 1.0
    ]
</rosparam>
<rosparam param="T_C2toI">
    [
    0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
    0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
    -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
    0.0, 0.0, 0.0, 1.0
    ]
</rosparam>
```


Refs: [1](https://github.com/rpng/open_vins/issues/130), [2](https://github.com/rpng/open_vins/issues/285)
