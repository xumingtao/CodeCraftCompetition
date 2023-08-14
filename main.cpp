#include <iostream> 
#include<algorithm>
#include <vector>
#include <fstream>
#include<sstream>
#include <string>
#include<map>
#include<cmath>
#include<time.h>
//#include"environment.h"
#include"InitParameters.h"
#define _CRT_SECURE_NO_WARNINGS
#define PI 3.1415926
#define RADUS 0.4
#define DangerCrashDIS 1.40
#define RobotNum 4
#define TotalFrame 9000
#define DT 0.02
#define Max_Speed 6.0
#define WzAngle PI//角度误差
#define ZWSpeed 0.5
using namespace std;
extern vector<int>NewAddRot;
extern vector<int>NewDetRot;
extern vector<WbSN> WbProductFull;//存储工作台物品已经生产好的列表
extern vector<WbSN>WbNeedProduct;
extern map<int, RobotSN>TaskRobot;//分配好了任务 去执行买或者卖的任务 
extern ArbiterData ArbFrame;
extern vector<RefRobotSN> RefRobotList;
extern int WbDataNum;
extern vector<WbSN>WBData_list;
double HangXiangAngle(double x1, double y1, double _angle, bool& Angle_Ok)//角度计算存在一定的问题
{
	double a = 0.0, b = 0.0;
	if (_angle < 0)
		a =2*PI+_angle;//将航向角转化为 0-2*PI
	else
		a = _angle;//当前航向角
	//if(atan())
	//b = acos(x1 / (sqrt(x1*x1 + y1 * y1)));
	if (x1 > 0 && y1 > 0)
		b = atan(y1 / x1);
	else if (x1 < 0 && y1>0)
		b = atan(abs(y1 / x1)) + PI / 2.0;
	else if (x1 < 0 && y1 < 0)
		b = atan(abs(y1 / x1)) + PI;
	else if (x1 > 0 && y1 < 0)
		b = atan((abs(y1 / x1))) + 3 * PI / 2.0;
	else if (x1 == 0)
	{
		if (y1 >= 0)
			b = PI / 2.0;
		else
			b = 3 * PI / 2.0;
	}
	else if (y1 == 0)
	{
		if (x1 >= 0)
			b = 0;
		else
			b = PI;
	}
	//b 目标角度
//	double c=_robot.robotPID.PID_realize(b, 2 * PI, a);
	double temp=b-a;// 下一时刻的角度与当前角度之差 再除以时间即得到角速度 

	if (fabs(temp) < WzAngle)
		Angle_Ok = true;
	else
		Angle_Ok = false;

	double temp1=temp;
	double angleSpeed;

	//_robot.robotPID.

	if (temp > PI)
		temp1 = temp - 2 * PI;
	if (temp < -PI)
		temp1 = temp + 2 * PI;
	if (temp1 / 0.02 > PI)
		return PI;
	if (temp1 / 0.02 < -PI)
		return -PI;
	angleSpeed = temp1 / 0.02;

/*	if (fabs(temp1) < PI / 25 && fabs(temp1) > PI / 50) 
	{
		angleSpeed = angleSpeed * (1 - exp(-fabs(temp1)));
	}*/
	/*if (fabs(temp1) < PI / 6) {
		angleSpeed = angleSpeed * (1 - exp(-fabs(temp1)));
	}*/
	return angleSpeed;
}

double _Dspeed(double _x1, double _y1)
{
	double s = sqrt(_x1*_x1 + _y1 * _y1);
	
	if (s / 0.02 > 6.0)
		return 6.0;
	//需要考虑往后行驶 ，碰壁时再考虑
	return s / 0.02;
}

bool IsNeedBuyOwning(vector<WbSN>_WbNeedProduct, int _buynum)
{
	for (int i = 0; i < _WbNeedProduct.size(); i++)
	{
			for (int j = 0; j < _WbNeedProduct[i].NowFrameNeedMat.size(); j++)
			{
				if (_WbNeedProduct[i].NowFrameNeedMat[j] == _buynum)
					return true;
			}
	}
	return false;
}
//用于判定距离时使用  发现当前帧需要原材料的工作台
vector<WbSN> FunNeedTempNum(vector<WbSN> _WbNeedProduct, int _neednum)
{
	vector<WbSN> _needTempnum;
	if (_WbNeedProduct.size() == 0)
		return _needTempnum;
	for (int i = 0; i < _WbNeedProduct.size(); i++)
	{
		bool flag = true;
		for (int j=0;j < _WbNeedProduct[i].NowFrameNeedMat.size()&&flag; j++)
		{
			if (_neednum == _WbNeedProduct[i].NowFrameNeedMat[j])
			{
				_needTempnum.push_back(_WbNeedProduct[i]);
				flag = false;
			}
		}
	}
	return _needTempnum;
}


bool WbIsNeedProductN(vector<WbSN> _WbNeedProduct, RefRobotSN _RefRobotList)
{
	if (_WbNeedProduct.size() == 0)
		return false;
	for (int i =0 ; i < _WbNeedProduct.size(); i++)
	{
		if (_WbNeedProduct[i].ID == _RefRobotList.MuBiaoWb.ID)
		{
			for (int j = 0; j < _WbNeedProduct[i].NowFrameNeedMat.size(); j++)
			{
				if (_RefRobotList.owning_ID == _WbNeedProduct[i].NowFrameNeedMat[j])
					return true;
			}
		}
	}
	return false;
}

//返回一个当前帧不需要的产品集合 然后在买的时候直接不买 
vector<int> NowFrameNoNeed(vector<WbSN>_WbNeedProduct)
{
	vector<int>Need;
	for (int i = 0; i < _WbNeedProduct.size(); i++)
	{
		for (int j = 0; j < _WbNeedProduct[i].NowFrameNeedMat.size(); j++)
		{
			vector< int >::iterator iter = std::find(Need.begin(), Need.end(), _WbNeedProduct[i].NowFrameNeedMat[j]);//查找当前帧下需要原材料的集合 如果不需要在当前帧不需要购买
			if (iter == Need.end())
				Need.push_back(_WbNeedProduct[i].NowFrameNeedMat[j]);
		}
	}
	vector<int>NoNeed;
	for (int i = 1; i <= 7; i++)
	{
		vector< int >::iterator iter = std::find(Need.begin(), Need.end(), i);
		if (iter == Need.end())
			NoNeed.push_back(i);
	}
	return NoNeed;//返回不需要的材料种类
}

//判断是否发生碰撞
//vector<bool>Robotcrash(RobotNum);

vector<bool> IsRobotcrash(vector<bool> _Robotcrash)
{
	for (int i = 0; i < RobotNum; i++)
		_Robotcrash[i] = false;
	for (int i = 0; i < RobotNum-1; i++)
	{
		for (int j = i + 1; j < RobotNum; j++)
		{
			double tempx = ArbFrame._robotsn[i].X_station - ArbFrame._robotsn[j].X_station;
			double tempy = ArbFrame._robotsn[i].Y_station - ArbFrame._robotsn[j].Y_station;
			double tempdis = sqrt(tempx*tempx + tempy * tempy);
			if (tempdis < DangerCrashDIS)//处在危险情况下 采取措施进行避障处理
			{
				_Robotcrash[i] = true;//表示二者发生碰撞
				_Robotcrash[j] = true;
			}
			//if(ArbFrame._robotsn)
		}
	}
	/*for (int i = 0; i < RobotNum; i++)
	{
		double x = ArbFrame._robotsn[i].X_station;
		double y = ArbFrame._robotsn[i].Y_station;
		if(x>48.5||x<1.5||y>48.5||y<0.7)
			_Robotcrash[i] = true;
	}*/
	return _Robotcrash;
}

bool CmpBuySell(StatusNum c1, StatusNum c2)
{
	return c1.ID < c2.ID;
}

vector<int> _BuySellOkListF( int length)
{
	vector<int>_BuySellOkList(ArbFrame.AllWbNum, -1);//
	for (int t = 0; t < RobotNum; t++)
	{
		if (RefRobotList[t].MatchOk == true)
		{
			_BuySellOkList[RefRobotList[t].MuBiaoWb.ID] = 1;
		}
	}
	return _BuySellOkList;
}

//vector<int>_BuySellOkList(ArbFrame.AllWbNum, -1);//

bool beginCmp(WbSN c1, WbSN c2)
{
	int t = c1.robotid;
	if (c1.disWbRobot[t] != c1.disWbRobot[t])
		return c1.disWbRobot[t] < c1.disWbRobot[t];
	if (c1.num != c2.num)
		return c1.num > c2.num;
}

vector<RefRobotSN> BeginMatch(vector<WbSN>_WBData_list, vector<RefRobotSN>_RefRobotList)
{
	vector<WbSN>Temp_WBData_list;
	for (int i = 0; i < _WBData_list.size(); i++)
	{
		if (_WBData_list[i].num <= 3)
			Temp_WBData_list.push_back(_WBData_list[i]);
	}

	for (int i = 0; i < _RefRobotList.size(); i++)
	{
		for (int j = 0; j < Temp_WBData_list.size(); j++)
		{
			double tempx = Temp_WBData_list[j].x- _RefRobotList[i].x_station;
			double tempy = Temp_WBData_list[j].y - _RefRobotList[i].y_station;
			Temp_WBData_list[j].disWbRobot[i] = sqrt(tempx*tempx + tempy * tempy);
			Temp_WBData_list[j].robotid = i;
		}
		if (Temp_WBData_list.size() != 0)
		{
			sort(Temp_WBData_list.begin(), Temp_WBData_list.end(), beginCmp);
			_RefRobotList[i].MuBiaoWb = Temp_WBData_list[0];
			RefRobotList[i].MatchOk = true;
			int tt = 0;
			bool flagbreak = true;
			for (int t = 0; t < _WBData_list.size()&& flagbreak; t++)
			{
				if (_WBData_list[t].ID == Temp_WBData_list[0].ID)
				{
					tt = t;
					flagbreak = false;
				}
			}
			vector<WbSN>::iterator it = _WBData_list.begin();
			_WBData_list.erase(it+tt);
		}		
	}
	return _RefRobotList;
}


int count_Frame = 10;
int main()
{
	readUntilOK(0);
	puts("OK");
	//地图内容读取 OK 工作台数据存储在 WBData_List  机器人数据存储在Robot_List
	fflush(stdout);
	int frameID;
	//clock_t start, end;
	ofstream f2("frameBuyNum.txt");

	//在此完成移动前匹配 

	RefRobotList = BeginMatch(WBData_list,RefRobotList);

	//vector<bool>BuySellOkList(WbDataNum);//
	while (scanf_s("%d", &frameID) != EOF)
	{
		readUntilOK(frameID);
		printf("%d\n", frameID);
		//memset(Robotcrash, false, sizeof(Robotcrash) / sizeof(bool));
		//判断是否发生碰撞
		vector<bool>Robotcrash(RobotNum,false);
		Robotcrash = IsRobotcrash(Robotcrash); //用来判定是否发生了碰撞
		WbNeedProduct = FindWbNeedRawMat();
		vector<int>NoNeedFrame = NowFrameNoNeed(WbNeedProduct);
		WbProductFull = FindWbIsFull(NoNeedFrame);//按照优先级进行  发现需要去买的工作台
	//	vector<int>BuySellOkList(ArbFrame.AllWbNum,-1);//
		vector<RefRobotSN>KJWbRobot; //判断机器人是否靠近工作台 此工作台是非目标工作台
		vector<StatusNum>BuySellNum;
		vector<int>BuySellOkList;
		//返回一个不需要的产品集合 然后在买的时候直接不买
		// 执行四个机器人任务调度分配算法
		for (int i = 0; i < RobotNum; i++)
		{
			BuySellOkList=_BuySellOkListF(ArbFrame.AllWbNum);
			//if (RefRobotList[i].MatchOk != true)
				//RefRobotList[i].MatchOk = false;
			if (RefRobotList[i].status == -1 && WbProductFull.size() != 0 && RefRobotList[i].MatchOk == false)
			{
				//如果存在7 就去买7 第一时间去买 
				vector<WbSN> TempBuy7;
				for (int tt = 0; tt < WbProductFull.size(); tt++)
				{
					WbProductFull[tt].robotid = i;//
					if (WbProductFull[tt].num == 7)
						TempBuy7.push_back(WbProductFull[tt]);
				}
				if (TempBuy7.size() != 0)
				{
					sort(TempBuy7.begin(), TempBuy7.end(), cmp);//之前提供的不存在效益 cmp按照距离 num优先级进行比较
					bool flag = true;
					int flag_int = 0; //用于删除数据文件标记地点
					for (int tt = 0; tt < TempBuy7.size()&&flag; tt++)
					{
						if (BuySellOkList[TempBuy7[tt].ID] == -1)
						{
						//	BuySellOkList[RefRobotList[i].MuBiaoWb.ID] = -1;
							RefRobotList[i].MuBiaoWb = TempBuy7[tt];
							BuySellOkList[RefRobotList[i].MuBiaoWb.ID] = 1;
					//		RefRobotList[i].robotPID.PID_init();
							flag_int = tt;
							flag = false;
							BuySellOkList[TempBuy7[tt].ID] = 1;
						}
					}
					if (!flag)
					{
						bool BreakFlag = true;
						for (int ti = 0; ti < WbProductFull.size()&& BreakFlag; ti++)
						{
							if (TempBuy7[flag_int].ID == WbProductFull[ti].ID)
							{
								vector<WbSN>::iterator k = WbProductFull.begin();
								WbProductFull.erase(k + ti);//必须将已分配好的数据清除 不然后续仍然会处理
								BreakFlag = false;
							}
						}
						RefRobotList[i].status = 1;
						RefRobotList[i].MatchOk = true; //此前不一定存在数据匹配
					}
				}//表示不存在7号物品去运输 
				else 
				{
					sort(WbProductFull.begin(), WbProductFull.end(), cmp);
					bool flagtf = true;
					int k = 0;
					for (int tt = 0; tt < WbProductFull.size() && flagtf; tt++)
					{
						if (BuySellOkList[WbProductFull[tt].ID] == -1)
						{
							RefRobotList[i].MuBiaoWb = WbProductFull[tt];
							BuySellOkList[RefRobotList[i].MuBiaoWb.ID] = 1;
						//	RefRobotList[i].robotPID.PID_init();
							k = tt;
							flagtf = false;
							BuySellOkList[WbProductFull[tt].ID] = 1;
						}
					}
					if (!flagtf)
					{
						vector<WbSN>::iterator k1 = WbProductFull.begin();
						WbProductFull.erase(k1+k);
						RefRobotList[i].status = 1;
						RefRobotList[i].MatchOk = true;
					}
				}
			}

			if (RefRobotList[i].status == 2 && WbNeedProduct.size() != 0 && RefRobotList[i].MatchOk == false)
			{
				bool _flag = false;
				RefRobotList[i] = MatchSellWb(WbNeedProduct, RefRobotList[i], _flag, BuySellOkList);
				if (_flag)
				{
					bool ZXFlag = true;
					for (int t = 0; t < WbNeedProduct.size() && ZXFlag; t++)
						if (RefRobotList[i].MuBiaoWb.ID == WbNeedProduct[t].ID)
						{
							vector<WbSN>::iterator k = WbNeedProduct.begin();
							WbNeedProduct.erase(k + t); //解决了可能存在任务分配重复的可能性
							ZXFlag = false;
							//break;
						}
					//RefRobotList[i].status = 2;
					RefRobotList[i].MatchOk = true; //只有删除掉 才是实现真正的匹配
				}
			}

			if (RefRobotList[i].MatchOk == true)
			{
				if (RefRobotList[i].status == 1)
				{
					double tempx = RefRobotList[i].MuBiaoWb.x - RefRobotList[i].x_station;
					double tempy = RefRobotList[i].MuBiaoWb.y - RefRobotList[i].y_station;
					double tempangle;//
					//int i = 0;
					bool _angleOk;
					if (Robotcrash[i])
					{
						tempangle = HangXiangAngle(tempx, tempy, ArbFrame._robotsn[i].TowardAngle , _angleOk)+PI/2.0;//	
					}
					else {
						tempangle = HangXiangAngle(tempx, tempy, ArbFrame._robotsn[i].TowardAngle, _angleOk);//
					}
					//printf("forward %d %lf\n", i, _Dspeed(tempx, tempy));//¿¿¿
					//printf("rotate %d %lf\n", i, tempangle);//¿¿
				//	double temp = sqrt(tempx*tempx + tempy * tempy);

					//在这里查询下是否有工作台需要这个要买的物品 如果不存在 不要买了
					//bool flagNeedBuy = IsNeedBuyOwning(WbNeedProduct, RefRobotList[0].MuBiaoWb.num);
					//RefRobotList[0].MatchOk = true;
					if (RefRobotList[i].KJWbId == RefRobotList[i].MuBiaoWb.ID/*temp <= RADUS*/)
					{
						bool flagNeedBuy = IsNeedBuyOwning(WbNeedProduct, RefRobotList[i].MuBiaoWb.num);//可能存在无法买入 8.9 的可能
						if (flagNeedBuy)//这里存在一个问题 无法确定已删除的工作台是否存在买入物品的可能性
						{
							StatusNum tempBS;
							tempBS.ID = i;
							tempBS.num = RefRobotList[i].MuBiaoWb.num;
							tempBS.BuyOk = true;
							BuySellOkList[RefRobotList[i].MuBiaoWb.ID] = -1;
							BuySellNum.push_back(tempBS);
							//tempBS.f
							//printf("buy %d\n", i);
							RefRobotList[i].MatchOk = false;
							RefRobotList[i].status = 2; //表示此时拥有物品！！！
							f2 << "1" << "  " << frameID << "    " << tempBS.num << endl;
							//执行买卖操作时，可以让其稍微降低一些速度 虽不至于直接降到0 但是可以慢慢减速降到
							printf("forward %d %lf\n", i, ZWSpeed);//¿¿¿
							printf("rotate %d %lf\n", i, WzAngle);//¿¿//逆时针旋转 
						}
						else //表示当前要买入的物品在全局中不存在需要购买的工作台
						{
							vector<WbSN> DisWbProductFull;//第二次分配 
							for (int t = 0; t < WbProductFull.size(); t++)
							{
								if (WbProductFull[t].num != RefRobotList[i].MuBiaoWb.num)
								{
									WbProductFull[t].robotid = i;
									DisWbProductFull.push_back(WbProductFull[t]);
									//RefRobotList[i].MuBiaoWb = WbProductFull[t];//这样做无法解决距离分配的问题 
								}
							}
							if (DisWbProductFull.size() != 0) 
							{
								sort(DisWbProductFull.begin(), DisWbProductFull.end(), cmp);
								BuySellOkList[RefRobotList[i].MuBiaoWb.ID] = -1;
								RefRobotList[i].MuBiaoWb = DisWbProductFull[0];
								BuySellOkList[RefRobotList[i].MuBiaoWb.ID] = 1;
								bool _tempangleOK;
								double tempx1 = RefRobotList[i].MuBiaoWb.x - RefRobotList[i].x_station;
								double tempy1 = RefRobotList[i].MuBiaoWb.y - RefRobotList[i].y_station;
								double tempangle1=HangXiangAngle(tempx1, tempy1, ArbFrame._robotsn[i].TowardAngle, _tempangleOK);
								
								if (_tempangleOK)
								{
									printf("forward %d %lf\n", i, _Dspeed(tempx1, tempy1));//¿¿¿
									printf("rotate %d %lf\n", i, tempangle1);
								}
								else {
									printf("forward %d %lf\n", i, ZWSpeed);//¿¿¿
									printf("rotate %d %lf\n", i, tempangle1);
								}//¿¿
							}
							else {//当前不一定与之前速度一样了
								if (_angleOk)
								{
									printf("forward %d %lf\n", i, _Dspeed(tempx, tempy));//¿¿¿
									printf("rotate %d %lf\n", i, tempangle);
								}
								else {
									printf("forward %d %lf\n", i, ZWSpeed);//¿¿¿
									printf("rotate %d %lf\n", i, tempangle);
								}
								//printf("forward %d %lf\n", i, RefRobotList[i].robotPID.PID_realize(0.0,8));//¿¿¿
								//printf("rotate %d %lf\n", i, RefRobotList[i].robotPID.PID_realize(0, 2 * PI));//¿¿ //或者规划一条路径 将其停往待停区
							}
						}
					}
					else
					{
						if (_angleOk)
						{
							printf("forward %d %lf\n", i, _Dspeed(tempx, tempy));//¿¿¿
							printf("rotate %d %lf\n", i, tempangle);
						}
						else {
							printf("forward %d %lf\n", i, ZWSpeed);//¿¿¿
							printf("rotate %d %lf\n", i, tempangle);
						}
						
						//printf("forward %d %lf\n", i,  _Dspeed(tempx, tempy));//¿¿¿
						//printf("rotate %d %lf\n", i, tempangle);//¿¿
					}
				}
				else if (RefRobotList[i].status == 2)
				{
					double tempx = RefRobotList[i].MuBiaoWb.x - RefRobotList[i].x_station;
					double tempy = RefRobotList[i].MuBiaoWb.y - RefRobotList[i].y_station;
					double tempangle;//

					bool _angleOK;
				//	int i = 0;
					if (Robotcrash[i])
					{
						tempangle = HangXiangAngle(tempx, tempy, ArbFrame._robotsn[i].TowardAngle , _angleOK)+PI/2.0;//   
					}
					else {
						tempangle = HangXiangAngle(tempx, tempy, ArbFrame._robotsn[i].TowardAngle, _angleOK);//
					}
					double temp = sqrt(tempx*tempx + tempy * tempy);
					//RefRobotList[0].MatchOk = true;
					if (RefRobotList[i].KJWbId== RefRobotList[i].MuBiaoWb.ID/*temp < RADUS*/)
					{
						bool IsSellOk = WbIsNeedProductN(WbNeedProduct, RefRobotList[i]);
						if (IsSellOk)
						{
							//检查一下 当前工作台是否仍然需要此材料 不需要的话 重新规划路线 如果没有其他工作台需要此材料了
							//执行destroy操作，去携带其他的材料 这一块要斟酌一下 是否考虑做个标志位 
							StatusNum tempBS;
							tempBS.ID = i;
							tempBS.SellOK = true;
							BuySellNum.push_back(tempBS);
							//printf("sell %d\n", i); //标记一下留作后续输出处理
							RefRobotList[i].MatchOk = false;
							RefRobotList[i].status = -1;
							BuySellOkList[RefRobotList[i].MuBiaoWb.ID] = -1;
							printf("forward %d %lf\n", i, ZWSpeed);//¿¿¿
							printf("rotate %d %lf\n", i, WzAngle);//¿¿//逆时针旋转 
						}
						else
						{
							if (WbNeedProduct.empty()) { /*等待处理 */ }
							else
							{
								vector<WbSN> _TempWbNeedProduct;
								for (int t = 0; t < WbNeedProduct.size(); t++)
								{
									bool flag1 = true;
									for (int tf = 0; tf < WbNeedProduct[t].NowFrameNeedMat.size() && flag1; tf++)
									{
										if (WbNeedProduct[t].NowFrameNeedMat[tf] == RefRobotList[i].owning_ID)
										{
											_TempWbNeedProduct.push_back(WbNeedProduct[t]);
											flag1 = false;
										}
									}
								}
								if (_TempWbNeedProduct.size() != 0)
								{
									for (int tf = 0; tf < _TempWbNeedProduct.size(); tf++)
									{
										double tempxf = _TempWbNeedProduct[tf].x - RefRobotList[i].x_station;
										double tempyf = _TempWbNeedProduct[tf].y - RefRobotList[i].y_station;
										double tempdisf = sqrt(tempxf*tempxf + tempyf * tempyf);
										_TempWbNeedProduct[tf].disWbRobot[RefRobotList[i].ID] = tempdisf;
										_TempWbNeedProduct[tf].robotid = RefRobotList[i].ID;
									}
									sort(_TempWbNeedProduct.begin(), _TempWbNeedProduct.end(), CmpMatchXz);

									BuySellOkList[RefRobotList[i].MuBiaoWb.ID] = -1;
									RefRobotList[i].MuBiaoWb = _TempWbNeedProduct[0];//需不需要执行删除操作呢 不用了吧 

									BuySellOkList[RefRobotList[i].MuBiaoWb.ID] =1;

									double tempXF = RefRobotList[i].MuBiaoWb.x- RefRobotList[i].x_station ;
									double tempYF = RefRobotList[i].MuBiaoWb.y - RefRobotList[i].y_station;
									bool _angleOkF;
									double angleF = HangXiangAngle(tempXF, tempYF, ArbFrame._robotsn[i].TowardAngle, _angleOkF);
									if (_angleOkF) {
										printf("forward %d %lf\n", i, _Dspeed(tempXF, tempYF));//¿¿¿
										printf("rotate %d %lf\n", i, angleF);//¿¿
									}
									else {
										printf("forward %d %lf\n", i,ZWSpeed);//¿¿¿
										printf("rotate %d %lf\n", i, angleF);//¿¿
									}
								}
								else
								{
									if (_angleOK)
									{
										printf("forward %d %lf\n", i, _Dspeed(tempx, tempy));//¿¿¿
										printf("rotate %d %lf\n", i, tempangle);//¿¿printf("forward %d %lf\n", i, _Dspeed(tempx, tempy));//¿¿¿
									}
									else {
										printf("forward %d %lf\n", i,ZWSpeed);//¿¿¿
										printf("rotate %d %lf\n", i, tempangle);//
									}
								}
							}
						}
					}
					else {
						if (_angleOK)
						{
							printf("forward %d %lf\n", i, _Dspeed(tempx, tempy));//¿¿¿
							printf("rotate %d %lf\n", i, tempangle);//¿¿printf("forward %d %lf\n", i, _Dspeed(tempx, tempy));//¿¿¿
						}
						else {
							printf("forward %d %lf\n", i, ZWSpeed);//¿¿¿
							printf("rotate %d %lf\n", i, tempangle);//
						}
					}
				}
			}
		}


		KJWbRobot = RefRobotList;//确定是取哪一种赋值方式 是之前赋值 还是之后赋值

		for (int i = 0; i < KJWbRobot.size(); i++)
		{
			if (KJWbRobot[i].KJWbId != -1 && KJWbRobot[i].KJWbId != KJWbRobot[i].MuBiaoWb.ID) //表明当前有靠近的目标 但目标又不为之前所分配的工作台
			{
				double tempx = KJWbRobot[i].x_station - ArbFrame._wbsn[KJWbRobot[i].KJWbId].x;
				double tempy = KJWbRobot[i].y_station - ArbFrame._wbsn[KJWbRobot[i].KJWbId].y;
				double tempdis = sqrt(tempx*tempx + tempy * tempy);
				if (KJWbRobot[i].status == 1 && tempdis < RADUS&&ArbFrame._wbsn[KJWbRobot[i].KJWbId].ProductStatus == 1)
				{
					//vector<int>TempNum=ArbFrame._wbsn[KJWbRobot[i].KJWbId].RawMaterial
					bool flag = true;
					for (int j = 0; j < WbProductFull.size() && flag; j++)
					{//靠近当前的工作台的ID与提供物品的ID相等 且num相等时可以执行买操作 
						if ((KJWbRobot[i].KJWbId == WbProductFull[j].ID) && KJWbRobot[i].MuBiaoWb.num == WbProductFull[j].num)
						{
							bool flagNeedBuy1 = IsNeedBuyOwning(WbNeedProduct, KJWbRobot[i].MuBiaoWb.num);
							if (flagNeedBuy1) 
							{
								StatusNum _BuySellNum;
								_BuySellNum.ID = i;
								_BuySellNum.BuyOk = true;
								_BuySellNum.num = WbProductFull[j].num;
							//	KJWbRobot[i].MatchOk = false;//表明当前买成功 工作台置于空闲状态
							//	KJWbRobot[i].status = -1;
								BuySellNum.push_back(_BuySellNum);
								BuySellOkList[KJWbRobot[i].MuBiaoWb.ID] = -1;
								KJWbRobot[i].MatchOk = false;
								KJWbRobot[i].status = 2; //表示此时拥有物品！！！
								f2 <<"2"<<"  "<< frameID << "    " << _BuySellNum.num << endl;
								flag = false;
							}
							//_BuySellNum.
						}
					}
				}
				else if (KJWbRobot[i].status == 2 && tempdis < RADUS/*&&ArbFrame._wbsn[KJWbRobot[i].KJWbId].ProductStatus == 0*/)//不一定非得需要的时候才去买
				{
					bool flag = true;
					for (int t = 0; t < WbNeedProduct.size() && flag; t++)
					{
						//for(int tt=0)
						if (KJWbRobot[i].KJWbId == WbNeedProduct[t].ID)
						{
							for (int tt = 0; tt < WbNeedProduct[t].RawMaterial.size() && flag; tt++)
							{
								if (KJWbRobot[i].owning_ID == WbNeedProduct[t].RawMaterial[tt])
								{
									StatusNum _BuySellNum;
									_BuySellNum.ID = i;
									_BuySellNum.SellOK = true;
									KJWbRobot[i].MatchOk = false;
									KJWbRobot[i].status = -1;
									BuySellNum.push_back(_BuySellNum);//会不会导致重复呢 谨慎处理 ！！！
									BuySellOkList[KJWbRobot[i].MuBiaoWb.ID] = -1;
									flag = false;
								}
							}
						}
					}
				}
			}
		}

		RefRobotList = KJWbRobot;

		//处理买卖程序 
		sort(BuySellNum.begin(), BuySellNum.end(), CmpBuySell);
		vector<StatusNum> TempBs;
		for (int i = 0; i < RobotNum; i++)
		{
			StatusNum temp;
			for (int j = 0; j < BuySellNum.size(); j++)
			{
				if (BuySellNum[j].ID == i)
				{
					if (BuySellNum[j].BuyOk == true)
					{
						temp.BuyOk = true;
						temp.num = BuySellNum[j].num;//这里需要买的话是买谁
					}
					if (BuySellNum[j].SellOK == true)
						temp.SellOK = true;
					if (BuySellNum[j].DestoryOK == true)
						temp.DestoryOK = true;
				}
			}
			temp.ID = i;
			TempBs.push_back(temp);
		}
		BuySellNum = TempBs;
		for (int i = 0; i < BuySellNum.size(); i++)
		{
			if (BuySellNum[i].BuyOk)
			{
				//判断 是否可以执行买操作
				int tempnum = BuySellNum[i].num;//要买的物品 
				//查找当前需要这个物品的工作台 
				vector<WbSN> NeedTempNum = FunNeedTempNum(WbNeedProduct, tempnum);
				if (NeedTempNum.size() != 0)
				{
					for (int tf = 0; tf < NeedTempNum.size(); tf++)
						NeedTempNum[tf].robotid = BuySellNum[i].ID;
					sort(NeedTempNum.begin(), NeedTempNum.end(), cmp);
					double tempx = RefRobotList[BuySellNum[i].ID].x_station - NeedTempNum[0].x;
					double tempy = RefRobotList[BuySellNum[i].ID].y_station - NeedTempNum[0].y;
					double tempdis = sqrt(tempx*tempx + tempy * tempy);
					if ((TotalFrame - frameID)*DT*Max_Speed > tempdis)
					{
						printf("buy %d\n", BuySellNum[i].ID);
					}
					else {
						RefRobotList[i].status = -1;
						RefRobotList[i].MatchOk = false;
					}
				}
				else
				{
					RefRobotList[i].status = -1;
					RefRobotList[i].MatchOk = false;
					//printf("buy %d\n", BuySellNum[i].ID);
				}
			}
			if(BuySellNum[i].SellOK)
				printf("sell %d\n", BuySellNum[i].ID);
			//if(BuySellNum[i].DestoryOK)
				//printf("destroy %d\n", BuySellNum[i].ID);
		}

		printf("OK\n");
		fflush(stdout);
	}
	return 0;
	//进行路径匹配操作 
}

