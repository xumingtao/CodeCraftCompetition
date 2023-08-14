#pragma once//基础配置文件 
#ifndef _InitParameters
#define _InitParameters
//#include"dwa.h"
#include<vector>
#include<map>
using namespace std;
typedef struct WorkbenchData {
	double Row_Coordinates;//横坐标
	double Col_Coordinates;//列坐标
	char num;//编号
	int Order;//工作台编号顺序
	int robotid;
	vector<double>disWbRobot;
}WBData;

typedef struct WbStatusNow {
	int num;
	double x;
	double y;
	int LeftFrame;//-1没有生产： 0 表示生产因输出格满而阻塞 ： >=0：表示剩余生产帧数。  
	int RawMaterialStatus;//原材料状态 表示当前所拥有的物品 用十进制数字表示 使用时需要将其转化为二进制 
	vector<int>RawMaterial;//
	vector<int>NowFrameNeedMat;//当前帧下需要的原材料
	int ProductStatus;//0 表示无 1 表示有
	vector<double> disWbRobot;//计算机器人与购物台的距离

	vector<double>Weight;//设置权重参数  便于后期分配

	int ID;

	int robotid;//目标机器人ID号

	int WbDestory ;//500帧内买不到物品 做销毁处理 

}WbSN;//工作台当前状态

typedef struct RobotStatusNow {
	int OrderRobot;//机器人序号
	int Wb_ID;//当前工作台的ID值  -1：表示当前没有处于任何工作台附近  [0,工作台总数-1] ：表示某工作台的下标，从 0 开始，
	//按输入顺序定。当前机器人的所有购买、出售行为均针对该工作台进行。
	int OwnWuping_ID;// 携带物品类型  0 表示未携带物品 1 - 7 表示对应物品。
	double TimeCostCoe;//时间价值系数 0.8-1.0 不携带物品时为0
	double CrashCostCoe;//碰撞价值系数
	double AngleSpeed;//角速度
	double x_Speed, y_Speed;//二维线速度
	double TowardAngle;//运动的朝向角度
	double X_station;//
	double Y_station;

	//bool IsOrHuoWU;//
	bool buyOK;//设置买入标志位
	bool sellOk;//设置卖出标志位
	bool destroy;//判断是否损坏物品
	bool AngleSpeedSetOk;//用于判断角速度是否设置好
	double DSpeed;//

	bool RenWuDivideOK;//判断机器人任务是否分配好的标志位
	int BindRobotWb;//将机器人与工作台进行绑定 
	WbSN MuBiaoWb;//表示机器人的目标位置
}RobotSN;


class pid {
public:
	float SetSpeed; //定义设定值
	float ActualSpeed; //定义实际值
	float err; //定义偏差值
	float err_last; //定义上一个偏差值
	float Kp, Ki, Kd; //定义比例、积分、微分系数
	float voltage; //定义电压值（控制执行器的变量）
	float integral; //定义积分值
public:
	void PID_init();
	float PID_realize(float speed, const double _num,float nowAngle);
};


typedef struct RobotRefresh {
	WbSN MuBiaoWb;//表示机器人的目标位置
	double x_station;//机器人当前坐标
	double y_station;
	double rawangle;//机器人当前角度
	double rawspeed;//机器人当前角速度
	double speed;//当前机器人速度


	int status;//-1 空闲 1 去买 2 去卖 
	int ID;
	int owning_ID;
	bool MatchOk;
	int KJWbId;
	bool destoryOK;//用来做损坏处理

	pid robotPID;



}RefRobotSN;//更新机器人的当前信息




typedef struct Arbiter {//每一帧判题器数据存储
	int FrameNum;//帧序号
	int MoneySum;//当前金钱数
	int AllWbNum;//当前所有的工作台数
	WbSN _wbsn[50];//最大值初始化
	RobotSN _robotsn[4];//机器人各个状态
}ArbiterData;//存储每一帧的数据

typedef struct StatusNum {// 当前输出时 机器人状态 
	int ID;
	int num;
	bool SellOK;
	bool BuyOk;
	bool DestoryOK;
};



//typedef struct _pid {
//	float SetSpeed; //定义设定值
//	float ActualSpeed; //定义实际值
//	float err; //定义偏差值
//	float err_last; //定义上一个偏差值
//	float Kp, Ki, Kd; //定义比例、积分、微分系数
//	float voltage; //定义电压值（控制执行器的变量）
//	float integral; //定义积分值
//
//	//void PID_init();
//
//	//float PID_realize(float speed);
//}pid;



//每次交互获取信息  
/*判题器传输过来的数据格式定义*/
string toBinary(int n);//十进制数转化为二进制状态
vector<int> Ten2Two(int data);
bool readUntilOK(int _frameId);//读取帧内数据是否可以实现
bool cmp(WbSN c1, WbSN c2);// 按照num从大到小排列 进行运算处理
bool cmp1(pair<WbSN, RobotSN> c1, pair<WbSN, RobotSN> c2);
vector<WbSN>FindWbIsFull(vector<int> _NoNeedBuy);//用来查询工作台的状态            计算出了当前工作台生产好的材料
vector<WbSN>FindWbNeedRawMat();//查找工作台需要何种原材料     计算出了当前工作台所需要的材料 
//vector<WbSN>FindWbisNeed();//用来查询工作台是否需要材料
//vector<RobotSN>FindRobotIsFull();//用来查询机器人的状态
vector<StatusNum> DivideRobotTask();//可以返回一个buy sell操作指令的集合
bool CmpStatus(StatusNum c1, StatusNum c2);
//vector<DWA*>DwaWbRob;
//bool cmpRobotId(StatusNum c1, StatusNum c2);//比较机器人ID大小，按照从小到大的顺序进行输出
//void PrintStageWbRobot(int _FrameID, vector<DWA*>_DwaWbRob, vector<DWA*>_buy_ok);//将计算好的数据打印输出
//vector<DWA*> FunDwaWbRob(vector<DWA*>&_DwaWbRob);//处理机器刷新函数
//bool CompareSameId(vector<DWA*>_DwaWbRob, DWA* _TempDwa);//比较是否出现两个机器人相同的情况
//void/*vector<DWA*>*/ ResourceMatch(vector<WbSN> _WbProductFull, vector<WbSN>  _FindWbIsNeed, vector<RobotSN> _RobotFreeList, vector<DWA*>&DwaWbRob);
RefRobotSN MatchSellWb(vector<WbSN> _WbNeedProduct, RefRobotSN _robot,bool& _flag,vector<int>& _BuySellOkList);

bool CmpMatchXz(WbSN c1, WbSN c2);

#endif // _InitParameters

