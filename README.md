# 说明
该仓库代码主要借鉴使用了https://github.com/MegviiRobot/CamOdomCalibraTool的代码实现(非常感谢其实现)  
知乎相关文章https://zhuanlan.zhihu.com/p/101727151  
原先代码用于camera & 车轮计标定，代码做了删除和修改，现在用于IMU & 车轮计标定  
同时基于现代化的C++ code对原先代码格式和命名进行了修改
## 编译：
使用build.sh进行编译  
修改DBUILD_SHARED_LIBS参数来控制生成动态库和静态库，默认使用静态图
使用CMAKE_BUILD_TYPE参数来控制生成Release代码或者Debug代码
更详细的CMAKE使用方式请参考官网
## 使用：
完成上一步编译操作后，在当前代码目录下运行run.sh，可以看到类似如下输出！[calibrateResult](documents/images/result.png)
和test_data/config_inner.yaml中的结果对比，相一致的话就是说明结果是符合预期的
标定的数据格式请参考:  
test_data/formatted_camera.txt  
test_data/formatted_odometry.txt
## 状态：
开发中