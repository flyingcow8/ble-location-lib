/***********************************************************************************************************************
* 版权信息：版权所有(c) 2015, 杭州海康威视数字技术股份有限公司，保留所有权利
*
* 文件名称：Bls.cpp
*
* 摘    要：海康威视蓝牙定位算法组件接口文件
*
*
* 当前版本：1.0.0
* 作    者：詹源
* 日    期：2015-06-01
* 备    注：初始版本
***********************************************************************************************************************/

#include "Bls.h"
#include "LevenbergMarquardt.h"

//检查某个基站信息是否已经存在target数组中
static int isInTarget(BLS_TRANSMISSION *transmission, BLS_TARGET *target)
{
    int i = 0;
    while (target[i].beacon) {
        if (transmission->beacon == target[i].beacon) {
            return i;       //检测到该基站已存在target数组的第i个元素
        }
        i++;
    }
    return -1;      //没有在target数组中找到该基站信息
}


//对手机的扫描数据进行预处理，将相同基站的多次扫描结果存到一个target对象中
static int transmissionsPreproccess(BLS_TRANSMISSION *transmissions[BLS_SCAN_FREQ], int *transCount, BLS_TARGET *target, int tarMax)
{
    int i, j;
    int seq;
    int beaconCount[tarMax];            //统计扫描到的每个iBeacon基站的次数
    int nextTar = 0;

    for (i = 0; i < tarMax; i++) {      //数组初始化
        beaconCount[i] = 0;
    }
    for (i = 0; i < BLS_SCAN_FREQ; i++ ) {
        for (j = 0; j < transCount[i]; j++ ) {
            seq = isInTarget(&transmissions[i][j], target);
            if (seq != -1) {                                                //检测到该基站存在target数组中，位置是seq
                target[seq].accuracy[beaconCount[seq]] = transmissions[i][j].accuracy;       //把该次扫描的accuracy值存到target对应beacon位置的accuracy数组中
                beaconCount[seq]++;
            } else {                                                    //检测到该基站第一次出现，尚位存在target数组中，把该基站信息添加到target数组中
                target[nextTar].beacon = transmissions[i][j].beacon;
                target[nextTar].accuracy[beaconCount[nextTar]] = transmissions[i][j].accuracy;
                beaconCount[nextTar]++;
                nextTar++;      //每扫描到一个新的iBeacon基站，nextTar加1
            }
        }
    }

    return nextTar;

}


//对扫描到的每个基站的accuracy数组进行滤波处理
static int targetFilter(BLS_TARGET *target, int tarCnt, BLS_FILTER *filter)
{
    int i, j, s = 0;
    double accCnt, sum;
    for (i = 0; i < tarCnt; i++) {
        accCnt = 0;
        for (j = 0; j < BLS_SCAN_FREQ; j++) {
            if (target[i].accuracy[j] >=0 && target[i].accuracy[j] != 1000) {       //统计每个accuracy数组中有效的扫描数据，剔除负值和初始化的1000值
                accCnt++;
            }
        }
        if (accCnt >  BLS_SCAN_FREQ / 2) {                      //只采纳扫描到的次数超过总的扫描次数一半的基站信息
            filter[s].location = target[i].beacon->location;
            sum = 0;
            for (j = 0; j < accCnt; j++) {
                sum += target[i].accuracy[j];
            }
            filter[s].accuracy = sum / accCnt;      //对accuracy做均值处理
            s++;
        }
    }

    return s;
}



//接口函数
int triangulation_nls(BLS_TRANSMISSION *transmissions[], int *transmissionCount, BLS_LOCATION *initLocation, BLS_LOCATION *location)
{
    int i, j, s = 0;
    for (i = 0; i < BLS_SCAN_FREQ; i++) {       //统计扫描的基站信息次数
        s += transmissionCount[i];
    }
    BLS_TARGET target[s];
    for (i = 0; i < s; i++) {       //target数组初始化
        target[i].beacon = NULL;
        for (j = 0; j < BLS_SCAN_FREQ; j++)
            target[i].accuracy[j] = 1000;
    }

    int targetCount = transmissionsPreproccess(transmissions, transmissionCount, target, s);    //对扫描数据预处理，返回基站个数

    BLS_FILTER filter[targetCount];
    int filterCount = targetFilter(target, targetCount, filter);    //对扫描数据进行滤波处理，返回有效基站个数


    if ( filterCount < 3 ) {                //有效基站个数小于3个，不满足三角定位的基本条件
        return HIK_BLS_TRANSMISSIONS_FEW;
    }
    else {
        Eigen::VectorXd x(2);
        x << initLocation->x, initLocation->y;

        Eigen::MatrixXd matrix(filterCount, 3);

        for (i = 0; i < filterCount; i++) {
            BLS_FILTER f = filter[i];

            Eigen::VectorXd t(3);
            t << f.location->x, f.location->y, f.accuracy;

            matrix.row(i) = t;
        }

        distance_functor functor(matrix, filterCount);
        Eigen::NumericalDiff<distance_functor> numDiff(functor);
        Eigen::LevenbergMarquardt<Eigen::NumericalDiff<distance_functor>,double> lm(numDiff);
//        lm.parameters.maxfev = 2000;
//        lm.parameters.xtol = 1.49012e-08;
//        lm.parameters.ftol = 1.49012e-08;
//        lm.parameters.gtol = 0;
//        lm.parameters.epsfcn = 0;
        Eigen::LevenbergMarquardtSpace::Status ret = lm.minimize(x);

//        printf("%f, %f, %d\n", x[0], x[1], (int)ret);

        if (ret == 1 || ret == 2 || ret == 3 || ret == 4) {
            location->x = x[0];
            location->y = x[1];
            location->z = 0;
            return HIK_BLS_LOCATION_OK;
        }
        else {
            return HIK_BLS_LOCATION_FAILED;
        }
    }
}
