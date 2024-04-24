### 1.介绍
这是一个基于 `ORBSLAM3` 的相机稠密重建算法,支持包括RGBD、双目相机的稠密重建,针孔相机和鱼眼相机都是支持的.

### 2.依赖
* Eigen3
* Pangolin 0.6版本
* PCL 1.8~1.10
* OpenCV >= 4.2

### 3.编译
当你满足上述依赖时即可
```shell
chmod +x build.sh
bash build.sh
```

### 4.示例
对于深度相机数据集
```shell
- root_dir
    - rgb
        - 1.jpg
        - 2.jpg
        ...
    - depth
        - 1.png
        - 2.png
        ...
```
上述的数据当你配置好`rgbdslam.yaml`文件后,即可运行
```shell
./MyExample/rgbd_slam /root_dir
```

对于双目数据集,格式如下
```shell
- root_dir
    - left
        - 1.jpg
        - 2.jpg
        ...
    - right
        - 1.png
        - 2.png
        ...
    - disp(optional)
        - 1.png
        - 2.png
        ...
```
可以利用`Euroc`[数据集](https://pan.baidu.com/s/1SjzAdgzRN1PjRmzsQYwrvA?pwd=kyan),示例代码的 `stereoslam.yaml` 或者 `stereoslam_disp.yaml`文件就是 `Euroc` 的配置文件,可以参照这个文件修改运行自己的数据集,即可运行
```shell
# 有视差图
./MyExample/stereoslam_disp root_dir/left root_dir/right root_dir/disp
# 无视差图
./MyExample/stereoslam_disp root_dir/left root_dir/right
```
