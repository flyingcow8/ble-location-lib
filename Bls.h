/***********************************************************************************************************************
* 版权信息：版权所有(c) 2015, 杭州海康威视数字技术股份有限公司，保留所有权利
*
* 文件名称：Bls.h
*
* 摘    要：海康威视蓝牙定位算法组件接口文件
*
*
* 当前版本：1.0.0
* 作    者：詹源
* 日    期：2015-06-01
* 备    注：初始版本
***********************************************************************************************************************/

#ifndef HIK_BLS_LIB_H
#define HIK_BLS_LIB_H


#ifdef __cplusplus
extern "C" {
#endif

#include <time.h>


#define BLS_MAJOR_VERSION       1           //主版本号
#define BLS_SUB_VERSION         0           //子版本号
#define BLS_REVISION_VERSION    0           //修正版本号

#define BLS_VER_YEAR    15                  //年
#define BLS_VER_MONTH   6                   //月
#define BLS_VER_DAY     1                   //日

#define BLS_SCAN_FREQ   5                   //进行一次定位的扫描次数

//返回状态码
#define HIK_BLS_LOCATION_OK         0       //定位成功
#define HIK_BLS_LOCATION_FAILED     1       //定位失败
#define HIK_BLS_TRANSMISSIONS_FEW   2       //有效扫描数据太少


/***********************************************************************************************************************
* 数据结构
***********************************************************************************************************************/

//笛卡尔坐标
typedef struct _BLS_LOCATION_ {
    double x;       //x坐标
    double y;       //y坐标
    double z;       //高度坐标
} BLS_LOCATION;

//iBeacon基站信息
typedef struct _BLS_BEACON_ {
    int major;              //iBeacon协议中的major值
    int minor;              //iBeacon协议中的minor值
    int measuredPower;      //iBeacon协议中的measuredPower值，对android有用，iOS不需要
    BLS_LOCATION *location; //iBeacon基站坐标
} BLS_BEACON;

//手机扫描到的iBeacon信息
typedef struct _BLS_TRANSMISSION_ {
    BLS_BEACON *beacon;     //基站信息
    time_t *timestamp;      //时间戳
    int rssi;               //接收信号强度，对android有用，iOS不需要
    double accuracy;        //手机与基站距离的粗略估计，对于iOS,该值非负；对于Android，该值为-1
} BLS_TRANSMISSION;

//对于若干组扫描数据进行预处理后的数据
typedef struct _BLS_TARGET_ {
    BLS_BEACON *beacon;                 //基站信息
    int rssi[BLS_SCAN_FREQ];            //rssi数组，用于保存每次扫描结果中来自该基站的rssi值
    double accuracy[BLS_SCAN_FREQ];     //accuracy数组，用于保存每次扫描结果中来自该基站的accuracy值
} BLS_TARGET;

//对扫描数据进行滤波处理后的数据
typedef struct _BLS_FILTER_ {
    BLS_LOCATION *location;     //基站位置
    double accuracy;            //手机与基站距离的粗略估计
} BLS_FILTER;



/***********************************************************************************************************************
* 接口函数
***********************************************************************************************************************/

/***********************************************************************************************************************
*  功  能: 蓝牙三角定位模块
*  参  数: *
*           transmissions       -I      iBeacon扫描结果指针数组
            transmissionCount   -I      每次扫描到的基站个数数组
            initLocation        -I      初始化手机坐标
            targetLocation      -O      手机定位坐标
*  返回值:  HIK_BLS_LOCATION_OK， 定位成功；
            HIK_BLS_TRANSMISSIONS_FEW， 有效扫描数据太少；
            HIK_BLS_LOCATION_FAILED，定位失败；
***********************************************************************************************************************/
int triangulation_nls(BLS_TRANSMISSION *transmissions[], int *transmissionCount, BLS_LOCATION *initLocation, BLS_LOCATION *location);


#ifdef __cplusplus
}
#endif

#endif
