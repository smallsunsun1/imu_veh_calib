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
(未完待续)
## 状态：