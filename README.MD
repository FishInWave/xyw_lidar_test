# 说明
该库是用于测试一些C++片段或者API的用法，建议与相应的语雀文档搭配使用（因为我比较懒，可能不会进行同步）

# 依赖
* PCL
* ROS
* GLOG
* CERES

各依赖的安装教程请自行百度，本仓库的CMakeLists会尽量保持全面（即便某些库完全没有被使用到），以便调试方便。

# 已有测试
1. GLOG的配置和使用
2. ROS的动态调参，注意句柄不可用是"/"，且动态调参的callback会在初始化时被调用一次，完成默认参数的配置。
3. 测试KDtree构建的耗时：testKDtree
4. 测试多线程加速的点云转换速度：testPointCloudTransform\
答：PCL和自己写的OMP耗时接近。
5. 测试transform会不会改变点云的序号：testTransformIndex\
答：不会。
6. 测试Eigen里的`4*4`矩阵和`3*1`向量的计算以及类型转换：testEigenVectorDir\
答：Isometry3d可以和vector3d进行运算，且float类型可以通过cast转换为double
7. 测试Eigen矩阵求绝对值：testEigenArrayAbs\
答：对矩阵元素进行操作可以考虑调用array()方法
8. 测试PCL库里点云的getVector4fMap方法：testGetVector4fMap\
答：Vector和Array只是类型的区别，数值上一样；4f还是3f，仅仅是出于齐次计算是否方便考虑，第4位总是1。
9. 测试高斯分布进行刚体变换后，其协方差是不是$RCR^T$:testGauss\
答：公式正确。
10. 由于Ceres出bug因此测试Eigen里基于泛型的类型初始化：testEigenTemplate
11. 测试Eigen的norm计算：testMartixNorm\
答：norm返回二范数值，normlized返回归一化后的m的拷贝，normlize对m本体进行归一化。
12. 测试小角近似里的$\alpha,\beta,\gamma$是轴角还是欧拉角：testAngleAxlesAndEulerAngle\
答：实验证明：小角近似定理是针对欧拉角的，而不是轴角，详见《语雀-刚体变换及雅可比矩阵求解》。理论原因：小角近似定理来自于旋转矩阵，其变量本身就是欧拉角。
13. 测试Ceres里的rotation的排列：testCeresTransform\
答：ceres::QuaternionToRotation里rotation是按行排列的。
14. 测试Cholesky分解的用法：testLLT\
答：$LL^T=U^TU$
15. 测试合同变换($P^TAP=B$，即正交变换)是否会影响协方差矩阵的正定性：testCovAndPositiveDefinite\
答：协方差矩阵其实只有半正定（见语雀）。合同变换不会影响其正定性（能否LLT分解），代码里有利用Eigen求cov的写法。
16. 为了LiTAMIN2debug而创造的类：testIsConverged()\
答：时间太久，收敛方法应该来自于PCL的GICP实现。
17. 可视化高斯分布：toSeeGaussDistributionShape()
18. 验证PCLRGBA的内部存储方式，以及该类型的使用方法：toTestPCLRGBA\
答：内部使用了Union。内部其实是按照BGR存储的，不过使用者其实不用关心。
19. 测试电脑是什么模式：toSeeWhichEnd\
答：一般是小端模式。代码核心思路：Union包含两个变量，令其中一个变量为1后，若另一变量也为1，说明低字节在低地址。小端比较符合C的想法。比如0x0001在小端模式下，低字节才是0x01。
20. 测试文件输出流用法：testOfstream
21. 点云拼接：testVectorTransform
22. 测试ros::NodeHandle的用法和ROS_DEBUG输出：testNodeHandle\
答：~会自动查询当前节点的名字。"hi"和"/hi"其实没什么差别，因为当前句柄默认是"/"。想要进行分级，最好使用二级句柄，输出为"/hh/second"。ROS的消息机制是过滤器，发是都发了，只是只有高于当前logger等级的信息才会在终端显示出来。使用ROS_DEBUG宏后若要看到数据，可以`rosrun rqt_logger_level rqt_logger_level`来调整节点的通知等级。

# 测试切换方式
取消相应函数的注释，然后重新编译运行即可。

# 修改记录
2021-7-26 22:30 添加了README.MD，并加入了对动态调参以及句柄的测试。