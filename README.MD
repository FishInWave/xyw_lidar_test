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
2. ROS的动态调参，注意句柄不可用是"/"，且动态调参的callback会在初始化时被调用一次，完成参数的配置,参数值从rosparam sever获取，若不存在，则使用cfg的默认值，因此有动态调参情况下，其实不需要使用nh.param，此外，关于命名空间，～是直接继承当前节点名字，为了匹配，launch里应当把rosparam file放到node里面。
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
答：norm返回矩阵F范数值，normlized返回归一化(除以norm_F)后的m的拷贝，normlize对m本体进行归一化。
12. ~~测试小角近似里的$\alpha,\beta,\gamma$是轴角还是欧拉角：testAngleAxlesAndEulerAngle\
答：实验证明：小角近似定理是针对欧拉角的，而不是轴角，详见《语雀-刚体变换及雅可比矩阵求解》。理论原因：小角近似定理来自于旋转矩阵，其变量本身就是欧拉角。~~ 小角情况下，轴角等于欧拉角。
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
23. 测试多线程lock
24. 测试vector内存回收\
答：clear，erase都无法释放内存，必须使用vector<T>().swap()
25. eigen_conversions使用时必须在CMakeLists里添加对这个库的依赖。
26. 测试cmake的include顺序
答： 无论是使用""还是<>，都优先访问了opt，因此如果自己要修改，且不想变更库文件，还是直接修改include的文件名比较好。
27. 测试tf2的转换失败了，测试了tf2::Quaternion的耗时，平均每个操作0.005ms，相比之下，double的赋值耗时是其百分之一。
28. `testGetPath`:测试C++和ROS的路径获取方式，其中ROS可以得到包的位置，getcwd是得到当前运行位置。
29. `cloud_callback` 确认了pcl到ros的类型转换流程：pointclouT-》pointcloud2-》ros
30. `testTimeSDK`: 经测试ros::Time和std::chrono::system_clock基本一致。
31. `testSlerp`: slerp中的t是other的系数，即q.slerp(1,p) = p。
32. `testMultispinner`： 经测试，使用Multispinner后，回调函数自动进行多线程调用；
33. `testKITTIsync`: 经测试，KITTI sync raw data的点云没有做运动补偿，且gt和sync的差别巨大，修正距离超过5cm的超过四分之三。
34. `testEigenAffineAndTransform`:Affine是4*3矩阵，可以调用matrix方法得到4*4矩阵，pcl的getTransformation方法的形参为rpy，相当于Rz*Ry*Rx;
35. `calculate`: 用于进行刚体变换
36. `testTictoc`: 测试Tictoc类的输出单位，验证是ms，duration默认的是s
37. `testSwitch`: 有return后，直接结束整个函数，因此不需要break；
38.  `testEnumClass`: 测试c++11新特性 enum class和switch的用法
39. `testSO3`: 验证fast-gicp里的SO3运算，其实等价于Eigen里轴角到四元数的转换
40. `testEigenForceTrans`: Eigen::Matrix可以用C类型的指针或者数组初始化
41. `testQuateRotate`: q单位化后和matrix可以置换，没必要做转换。
42. `测试类别和计算速度`： O3优化下，点云变换使用数学表达式会比调pcl库更快。
43. `测试std::map`： 遍历删除有特殊的for写法，不可以用range for，插入根据使用的重载版本，返回值也有不同
44. `测试Cov的旋转`： 同9
45. `测试noalias`： noalias只可以用于矩阵乘法的赋值（不包含拷贝构造函数）。
# 测试切换方式
取消相应函数的注释，然后重新编译运行即可。
运行：
```
roslaunch xyw_lidar_test xyw_start.launch
```