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