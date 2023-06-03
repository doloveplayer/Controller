# TraditionalController

## 简单pid使用

初始化&使用方法

```
    PidController::BaseFactors_t bf1 = {0.3, 0.5, 0, 100, 200, 80, 1};
    PidController::SimplePidController Spid(PidController::PID_POSITION);
    spid.PidInit(bf1);
    spid.PidCalc(.....);
```

## 分段pid使用

初始化&使用方法

```
    PidController::BaseFactors_t bf1 = {0.3, 0.5, 0, 100, 200, 80, 1};
    PidController::BaseFactors_t bf2 = {0.2, 0.2, 0, 100, 200, 80, 1};
    PidController::BaseFactors_t bf3 = {0.1, 0.5, 0, 100, 200, 80, 1};
    std::vector<PidController::BaseFactors_t> bf={bf1,bf2,bf3};//多段pid参数
    std::vector<PidController::Segment_t> seg={{2000,20},{20,5},{5,-2000}};//分段区间

    //初始化
    PidController::SegmentPidController Segpid(3,PidController::PID_POSITION);
    spid.PidInit(bf,seg);
    
    //使用
    spid.PidSegmentCalc(....);
```

## 其他功能

初始化&使用方法

```
//前馈控制
   std::vector<PidController::Segment_t> seg={{2000,20},{20,5},{5,-2000}};
   std::vector<fp32> out = {20,10,5};
   spid.advanced_factors_.SegmentForwardFeedInit(seg,out);
//变速积分
   fp32 maxi =100;
   fp32 mini =10;
   spid.advanced_factors_.VariableIntergralInit(maxi,mini);
//梯形积分
   spid.advanced_factors_.trapezium_integral_.flag =1;
    
```

## 模糊控制器

初始化&使用方法

```
    //先初始化参数的幅度限制
    TraditionalController::FuzzyFactorRange_t ffr = {{10, -6},
                                                     {10, -6},
                                                     {10, -6},
                                                     {50, -50},
                                                     {50, -50}
    };
    //然后就像普通pid初始化那样使用
    TraditionalController::FuzzyPidController Fpid(TraditionalController::PID_POSITION);
    Fpid.PidInit(pid1, ffr);
    
    //调试方法
    /*调整pid的规则表 IF e NB AND er NB THEN P .....*/
    /*其中规则表竖坐标代表 e 对应的模糊量 横坐标代表 er对应的模糊量*/
    
    /*调整权重来获得直观效果 前提基于合理的pid规则*/
    /**给的权重越大作用效果越明显**/
    #define KpRight 0.001
    #define KiRight 0.001
    #define KdRight 0.0005
    
    /*调整隶属度函数 以及隶属度函数的宽度*/
    #define MEMBERSHIP_STEP 4.0f //隶属度步长 步长越大 越小消耗算力 但是越精确
    #define triangular 0//三角
    #define trapezoidal 1//梯形
    #define gaussian 2//高斯
    
    /*其中隶属度函数类型的选择对控制效果改善并不大 主要还是控制步长（宽度）*/

    
```