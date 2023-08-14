#include"InitParameters.h"
//#include"dwa.h"
#include <iostream> 
#include<algorithm>
#include <vector>
#include <fstream>
#include<sstream>
#include <string>
#include<map>
#include<cmath>
#define pi 3.1415926
ofstream f1("data.txt");
ArbiterData ArbFrame;//每次传来的数据放在此处
bool InitData = false;//用来判定是否初始化获取了数据 InitData: false 未初始化， true 初始化完成
int WbDataNum = 0;
vector<int>NewAddRot;
vector<int>NewDetRot;

vector<WbSN>WBData_list;//工作台坐标数据的集合
vector<WorkbenchData>Robot_list;//机器人坐标数据集合

map<int, RobotSN>FreeRobot;//key 是指机器人ID RobotSN 是指机器人状态数据
map<int, RobotSN>TaskRobot;//分配好了任务 去执行买或者卖的任务 
map<int, RobotSN>BuyTaskRobot;//执行买操作的机器人集合
map<int, RobotSN>SellTaskRobot;//执行卖操作的机器人集合
vector<WbSN> WbProductFull;//存储工作台物品已经生产好的列表
vector<WbSN>WbNeedProduct;
vector<RobotSN> RobotFreeList;
vector<pair<WbSN, RobotSN>>StageWbRob;//将机器人与工作台一一对应进行运动

vector<RefRobotSN>RefRobotList;//全局变量 机器人

string toBinary(int n)//十进制数转化为二进制状态
{
	string r;
	while (n != 0) {
		r += (n % 2 == 0 ? "0" : "1");
		n /= 2;
	}
	return r;
}

vector<int> Ten2Two(int data)
{
	vector<int> dataf;
	string r = toBinary(data);
	for (int i = 0; i < r.size(); i++)
	{
		if (r[i] == '1')
			dataf.push_back(i);
	}
	return dataf;
}

bool readUntilOK(int _frameID)//读取帧内数据是否可以实现
{
	if (!InitData)
	{
		char line[1024];
		int count_col = 0;//   用于判定是第几行数据
		int count_num_wb = -1;//工作台序号
		int count_num_robot = -1;//机器人序号
		while (fgets(line, sizeof line, stdin))
		{
			f1 << line << endl;
			if (line[0] == 'O' && line[1] == 'K')//最后一行数据标志位 结束
			{
				WbDataNum = WBData_list.size();//获取工作台数量
				InitData = true;
				return true;
			}
			for (int i = 0; i < sizeof line; i++)
			{
				if (line[i] <= '9'&&line[i] >= '1')
				{
					count_num_wb++;
				//	WBData temp_work;
					WbSN temp_work;
					temp_work.num = line[i];
					temp_work.y = 49.75 - 0.5*count_col;
					temp_work.x = 0.25 + 0.5*i;
					temp_work.ID = count_num_wb;
					WBData_list.push_back(temp_work);
				}
				else if (line[i] == 'A')
				{
					count_num_robot++;
					WBData temp_robot;
					temp_robot.num = line[i];
					temp_robot.Col_Coordinates = 49.75 - 0.5*count_col;
					temp_robot.Row_Coordinates = 0.25 + 0.5*i;
					temp_robot.Order = count_num_robot;
					Robot_list.push_back(temp_robot);

					RefRobotSN tempRefRob;
					tempRefRob.ID = count_num_robot;
					tempRefRob.status = -1;
					tempRefRob.MatchOk = false;
					tempRefRob.x_station= 0.25 + 0.5*i;
					tempRefRob.y_station= 49.75 - 0.5*count_col;
					tempRefRob.robotPID.PID_init(); //将PID执行初始化
					RefRobotList.push_back(tempRefRob);//获取全局变量
					//for(int t=0)
				}
			}
			count_col++;
			//do something
		}
	}
	else
	{
		int count_col = 0;
		string str;
		vector<double> data;
		//键入ctrl+Z，才能跳出循环
		bool FLAG = true;
		f1 << _frameID << "  ";
		while (getline(cin, str))
		{
			f1 << str << endl;
			istringstream is(str);
			string s;
			while (is >> s)
			{
				if (s != "OK")
					data.push_back(stod(s));
				else
					FLAG = false;
			}
			if (!FLAG)
				break;
		}
		//cout << "读取结束!" << endl;
		//ArbFrame.FrameNum = data[0];
		ArbFrame.MoneySum = data[0];
		ArbFrame.AllWbNum = data[1];
		int countNUmWb = 0;
		int Flag_data_num_wb = 2 + 6 * ArbFrame.AllWbNum;
		for (int i = 2; i < Flag_data_num_wb; i = i + 6)
		{
			WbSN TempWb;
			TempWb.num = data[i];
			TempWb.x = data[i + 1];
			TempWb.y = data[i + 2];
			TempWb.LeftFrame = data[i + 3];
			TempWb.RawMaterialStatus = data[i + 4];

			TempWb.RawMaterial = Ten2Two(TempWb.RawMaterialStatus);

			TempWb.ProductStatus = data[i + 5];
			TempWb.WbDestory = 500;
			for (int i = 0; i < 4; i++)
				TempWb.disWbRobot.push_back(0);//初始化4个0 ，后面在进行计算求解
			for (int i = 0; i < 4; i++)
				TempWb.Weight.push_back(0);
			TempWb.ID = countNUmWb;
			ArbFrame._wbsn[countNUmWb] = TempWb;
			countNUmWb++;
		}
		int countNumRobot = 0;
		/*
	int Wb_ID;//当前工作台的ID值  -1：表示当前没有处于任何工作台附近  [0,工作台总数-1] ：表示某工作台的下标，从 0 开始，
	//按输入顺序定。当前机器人的所有购买、出售行为均针对该工作台进行。
	int OwnWuping_ID;// 携带物品类型  0 表示未携带物品 1 - 7 表示对应物品。
	double  TimeCostCoe;//时间价值系数 0.8-1.0 不携带物品时为0
	double CrashCostCoe;//碰撞价值系数
	double AngleSpeed;//角速度
	double x_Speed,y_Speed;//二维线速度
	double TowardAngle;//运动的朝向角度
	double X_station;//
	double Y_station;
		*/
		for (int i = Flag_data_num_wb; i < data.size(); i += 10)
		{
			RobotSN TempRobot;
			TempRobot.OrderRobot = countNumRobot;//获取机器人序号
			TempRobot.Wb_ID = data[i];
			TempRobot.OwnWuping_ID = data[i + 1];
			TempRobot.TimeCostCoe = data[i + 2];
			TempRobot.CrashCostCoe = data[i + 3];
			TempRobot.AngleSpeed = data[i + 4];
			TempRobot.x_Speed = data[i + 5];
			TempRobot.y_Speed = data[i + 6];
			TempRobot.DSpeed = sqrt(TempRobot.x_Speed*TempRobot.x_Speed + TempRobot.y_Speed*TempRobot.y_Speed);//获取得到线速度大小
			TempRobot.TowardAngle = data[i + 7];
			TempRobot.X_station = data[i + 8];
			TempRobot.Y_station = data[i + 9];
			TempRobot.buyOK = false;
			TempRobot.sellOk = false;
			TempRobot.destroy = false;//留到后续进行优化设计
			ArbFrame._robotsn[countNumRobot] = TempRobot;

			RefRobotList[countNumRobot].x_station = data[i + 8];
			RefRobotList[countNumRobot].y_station = data[i + 9];
			RefRobotList[countNumRobot].rawspeed = TempRobot.AngleSpeed;
			RefRobotList[countNumRobot].speed = TempRobot.DSpeed;
			RefRobotList[countNumRobot].KJWbId = TempRobot.Wb_ID;
			if (TempRobot.OwnWuping_ID != 0)
			{
				RefRobotList[countNumRobot].status = 2;
				RefRobotList[countNumRobot].owning_ID = TempRobot.OwnWuping_ID;
			}
			countNumRobot++;
		}
		return true;
	}
}

bool cmp(WbSN c1, WbSN c2)// 按照num从大到小排列 进行运算处理
{
	int t = c1.robotid;
//	if (c1.Weight[t] != c2.Weight[t])
//		return c1.Weight[t] > c2.Weight[t];
	if (c1.disWbRobot[t] != c2.disWbRobot[t])
		return c1.disWbRobot[t] < c2.disWbRobot[t];
	if (c1.num != c2.num)
		return c1.num> c2.num;
	/*if (c1.num< c2.num)
		return true;*/
	//if(c1.)
	//return false;
}

bool cmp1(pair<WbSN, RobotSN> c1, pair<WbSN, RobotSN> c2)
{
	return c1.second.OrderRobot < c2.second.OrderRobot;//执行从小到大排序方式
}


vector<WbSN>FindWbIsFull(vector<int> _NoNeedBuy)//用来查询工作台的状态
{
	//if(_NoNeedBuy.size()!=0){ }
	vector<WbSN> _WbProductFull;

	for (int i = 0; i < ArbFrame.AllWbNum; i++)
	{
		if (ArbFrame._wbsn[i].ProductStatus == 1)
		{
			vector< int >::iterator iter = std::find(_NoNeedBuy.begin(), _NoNeedBuy.end(), ArbFrame._wbsn[i].num);
			if (iter == _NoNeedBuy.end())
			{
				double tempx, tempy, tempangle;
				for (int t = 0; t < 4; t++)
				{
					tempx = ArbFrame._wbsn[i].x - RefRobotList[t].x_station;
					tempy = ArbFrame._wbsn[i].y - RefRobotList[t].y_station;
					tempangle = sqrt(tempx*tempx + tempy * tempy);//
					ArbFrame._wbsn[i].disWbRobot[t] = tempangle;//
					ArbFrame._wbsn[i].Weight[t] = 0.2*ArbFrame._wbsn[i].num +0.8* (7.0 - tempangle / 10.0); //距离权重分配方案
				}
				_WbProductFull.push_back(ArbFrame._wbsn[i]);//将产品格状态满的工作台筛选出来，便于后续方案分配
			}
		}
	}
	//sort(_WbProductFull.begin(), _WbProductFull.end(), cmp);//先按照num从大到小进行排序 
	return _WbProductFull;
}


vector<WbSN>FindWbNeedRawMat()//查找工作台需要何种原材料
{
	vector<WbSN>TempNeed;
	for (int i = 0; i < ArbFrame.AllWbNum; i++)
	{
		if (ArbFrame._wbsn[i].num > 3)
		{
			for (int t = 0; t < 4; t++)
			{
				double tempx = ArbFrame._wbsn[i].x - ArbFrame._robotsn[t].X_station;
				double tempy = ArbFrame._wbsn[i].y - ArbFrame._robotsn[t].Y_station;
				double tempDis = sqrt(tempx*tempx + tempy * tempy);
				ArbFrame._wbsn[i].disWbRobot[t] = tempDis;
			}

			switch (ArbFrame._wbsn[i].num) 
		{
			case 4:
			{
				int temp = ArbFrame._wbsn[i].RawMaterialStatus ^ 6;//异或操作
				if (temp != 0)
				{
					vector<int>NeedIdRaw = Ten2Two(temp);//NeedIdRaw 即为所需要的原材料数
					ArbFrame._wbsn[i].NowFrameNeedMat = NeedIdRaw;
					TempNeed.push_back(ArbFrame._wbsn[i]);
				}
				//if(ArbFrame._wbsn[i].RawMaterial.size()!=2)
				break;
			}
			case 5:
			{
				int temp = ArbFrame._wbsn[i].RawMaterialStatus ^ 10;//异或操作
				if (temp != 0)
				{
					vector<int>NeedIdRaw = Ten2Two(temp);//NeedIdRaw 即为所需要的原材料数
					ArbFrame._wbsn[i].NowFrameNeedMat = NeedIdRaw;
					TempNeed.push_back(ArbFrame._wbsn[i]);

				}
				break;
			}
			case 6:
			{
				int temp = ArbFrame._wbsn[i].RawMaterialStatus ^ 12;//异或操作
				if (temp != 0)
				{
					vector<int>NeedIdRaw = Ten2Two(temp);//NeedIdRaw 即为所需要的原材料数
					ArbFrame._wbsn[i].NowFrameNeedMat = NeedIdRaw;
					TempNeed.push_back(ArbFrame._wbsn[i]);
				}
				break;
			}
			case 7:
			{
				int temp = ArbFrame._wbsn[i].RawMaterialStatus ^ 112;//异或操作
				if (temp != 0)
				{
					vector<int>NeedIdRaw = Ten2Two(temp);//NeedIdRaw 即为所需要的原材料数
					ArbFrame._wbsn[i].NowFrameNeedMat = NeedIdRaw;
					TempNeed.push_back(ArbFrame._wbsn[i]);
				}
				break;
			}
			case 8:
			{
				int temp = ArbFrame._wbsn[i].RawMaterialStatus ^ 128;//异或操作
				if (temp != 0)
				{
					vector<int>NeedIdRaw = Ten2Two(temp);//NeedIdRaw 即为所需要的原材料数
					ArbFrame._wbsn[i].NowFrameNeedMat = NeedIdRaw;
					TempNeed.push_back(ArbFrame._wbsn[i]);
				}
				break;
			}
			default:
			{
				int temp = ArbFrame._wbsn[i].RawMaterialStatus ^ 240;//异或操作
				if (temp != 0)
				{
					vector<int>NeedIdRaw = Ten2Two(temp);//NeedIdRaw 即为所需要的原材料数
					ArbFrame._wbsn[i].NowFrameNeedMat = NeedIdRaw;
					TempNeed.push_back(ArbFrame._wbsn[i]);
				}
			}
		}
			//获取工作台ID 以及其当前状态所需的原材料，判断出来当前需要材料的工作台 以及需要什么原材料 
		}
	}
	return TempNeed;//需要的原材料数
}
bool CmpMatch(WbSN c1, WbSN c2)
{
	int t = c1.robotid;
	if (c1.disWbRobot[t] != c2.disWbRobot[t])
		return c1.disWbRobot[t] < c2.disWbRobot[t];
	if (c1.num != c2.num)
		return c1.num > c2.num;
	if (c1.ID != c2.ID)
		return c1.ID < c2.ID;
}

bool CmpMatchXz(WbSN c1, WbSN c2)
{
	int t = c1.robotid;
	if (c1.num != c2.num)
		return c1.num > c2.num;
	if (c1.disWbRobot[t] != c2.disWbRobot[t])
		return c1.disWbRobot[t] < c2.disWbRobot[t];
	if (c1.ID != c2.ID)
		return c1.ID < c2.ID;//解释了为什么小车会舍近求远的运输货物 应将距离都计算出来
}

RobotSN  MatchBuyWbRobot(vector<WbSN> _WbProductFull,RobotSN _robot)
{
	int t = _robot.OrderRobot;
	for (int i = 0; i < _WbProductFull.size(); i++)
	{
		double tempx = _WbProductFull[i].x - _robot.X_station;
		double tempy = _WbProductFull[i].y - _robot.Y_station;
		_WbProductFull[i].disWbRobot[t] = sqrt(pow(tempx, 2) + pow(tempy, 2));
		_WbProductFull[i].robotid = t;
	}
	sort(_WbProductFull.begin(), _WbProductFull.end(), CmpMatch);
	_robot.MuBiaoWb = _WbProductFull[0];//
	_robot.RenWuDivideOK = true;
	return _robot;
}

RefRobotSN MatchSellWb(vector<WbSN> _WbNeedProduct, RefRobotSN _robot,bool& _flag, vector<int>&_BuySellOkList)//去卖的问题解决了
{
	vector<WbSN>temp;
	for (int i = 0; i < _WbNeedProduct.size(); i++)
	{
		bool BreakFlag = true;
		for (int j = 0; j < _WbNeedProduct[i].NowFrameNeedMat.size() && BreakFlag; j++)
		{
			if (_WbNeedProduct[i].NowFrameNeedMat[j] == _robot.owning_ID)
			{
				double tempx = _WbNeedProduct[i].x - _robot.x_station;
				double tempy = _WbNeedProduct[i].y - _robot.y_station;
				double temp_dis = sqrt(tempx*tempx + tempy * tempy);
				_WbNeedProduct[i].disWbRobot[_robot.ID] = temp_dis;
				_WbNeedProduct[i].robotid = _robot.ID;
				temp.push_back(_WbNeedProduct[i]);
				BreakFlag = false;
			}

		}
	}
	if (temp.size() != 0)
	{
		sort(temp.begin(), temp.end(), CmpMatch);

		bool flagList = true;
		for (int i = 0; i < temp.size()&& flagList; i++)
		{
			if (_BuySellOkList[temp[i].ID] == -1)
			{
				_robot.MuBiaoWb = temp[i];
				_flag = true;
				flagList = false;
				_BuySellOkList[temp[i].ID] = 1;
			}
		}
		if (flagList)
		{
			//_robot.MuBiaoWb = temp[0];
			_flag = false;
		}
	}
	else 
	{
		_flag=false;
		//	_robot.destoryOK = true;//
	}//匹配成功
	 //_robot.RenWuDivideOK = true;
	//SellTaskRobot[_robot.OrderRobot] = _robot;
	return _robot;
}



RobotSN MatchSellWb(vector<WbSN> _WbNeedProduct, RobotSN _robot)//去卖的问题解决了
{
	vector<WbSN>temp;
	for (int i = 0; i < _WbNeedProduct.size(); i++)
	{
		bool BreakFlag = true;
		for (int j = 0; j < _WbNeedProduct[i].NowFrameNeedMat.size()&& BreakFlag; j++)
		{
			if (_WbNeedProduct[i].NowFrameNeedMat[j] == _robot.OwnWuping_ID)
			{
				double tempx = _WbNeedProduct[i].x - _robot.X_station;
				double tempy = _WbNeedProduct[i].y - _robot.Y_station;
				double temp_dis = sqrt(tempx*tempx + tempy * tempy);
				_WbNeedProduct[i].disWbRobot[_robot.OrderRobot] = temp_dis;
				temp.push_back(_WbNeedProduct[i]);
				BreakFlag = false;
			}

		}
	}
	if (_WbNeedProduct.size() != 0) 
	{
		sort(_WbNeedProduct.begin(), _WbNeedProduct.end(), CmpMatch);
		_robot.MuBiaoWb = _WbNeedProduct[0];//匹配成功
		_robot.RenWuDivideOK = true;
		SellTaskRobot[_robot.OrderRobot] = _robot;
	}
	return _robot;
}

bool CountDistanceWbRot(RobotSN rob)
{
	double tempx = rob.X_station - rob.MuBiaoWb.x;
	double tempy = rob.Y_station - rob.MuBiaoWb.y;
	double dis = sqrt(tempx*tempx + tempy * tempy);
	if (dis < 0.4)
		return true;
	else
		return false;
}

bool CmpStatus(StatusNum c1, StatusNum c2)
{
	return c1.ID < c2.ID;
}


//进行机器人任务划分
vector<StatusNum> DivideRobotTask()//可以返回一个buy sell操作指令的集合
{
	vector<RobotSN>TempISTask;//初步判定有任务的机器人
	vector<RobotSN>TempISNoTask;//初步判定的没有任务的机器人
	vector<StatusNum>ID_BSRot;//机器人集合的ID
	for (int i = 0; i < 4; i++)
	{
		if (ArbFrame._robotsn[i].OwnWuping_ID == 0)//表示此机器人未分配
		{
			TempISNoTask.push_back(ArbFrame._robotsn[i]);
		}
		else
			TempISTask.push_back(ArbFrame._robotsn[i]);
	}
	for (int i = 0; i < TempISNoTask.size(); i++)
		{
			if (BuyTaskRobot.find(TempISNoTask[i].OrderRobot) == BuyTaskRobot.end()&& WbProductFull.size() != 0)//等操作就完了
			{
				//TempISNoTask[i].RenWuDivideOK = true;
				RobotSN temp = MatchBuyWbRobot(WbProductFull, TempISNoTask[i]);//用来匹配工作台 实现买关系匹配
				BuyTaskRobot[TempISNoTask[i].OrderRobot] = temp;//加入机器人 
				TaskRobot[TempISNoTask[i].OrderRobot] = temp;
				NewAddRot.push_back(TempISNoTask[i].OrderRobot);
				vector<WbSN>::iterator k = WbProductFull.begin();
				WbProductFull.erase(k);
			}
			else if(BuyTaskRobot.size()!=0)
			{
				//查看分配的任务是否要完成了 通过标志位反应出来
				//Temp
				if (CountDistanceWbRot(BuyTaskRobot[TempISNoTask[i].OrderRobot]) == true)
				{
					BuyTaskRobot[TempISNoTask[i].OrderRobot].buyOK = true;//表示此机器人在当前帧可以执行买指令操作
					StatusNum SatTemp;
					SatTemp.ID = TempISNoTask[i].OrderRobot;
					SatTemp.BuyOk = true;
					SatTemp.SellOK = false;
					ID_BSRot.push_back(SatTemp);
					BuyTaskRobot.erase(TempISNoTask[i].OrderRobot);//将buy指令从 工作机器人中删除
					//没有做存在判定 存在问题
					TaskRobot.erase(TempISNoTask[i].OrderRobot);
					NewDetRot.push_back(TempISNoTask[i].OrderRobot);
					//并在此执行删除操作 做标记记下 当前机器人ID号 
				}
				//查看帧数 进行处理 
			}
			//BuyTaskRobot
			//if(BuyTaskRobot.find)
		}

	for (int i = 0; i < TempISTask.size(); i++)
	{
		if (TempISTask[i].RenWuDivideOK != true&& WbNeedProduct.size()!=0)
		{
			RobotSN temp = MatchSellWb(WbNeedProduct, TempISTask[i]);
			SellTaskRobot[TempISTask[i].OrderRobot] = temp;
			TaskRobot[TempISTask[i].OrderRobot] = temp;
			NewAddRot.push_back(TempISTask[i].OrderRobot);
			vector<WbSN>::iterator k = WbNeedProduct.begin();
			WbNeedProduct.erase(k);
			//WbNeedProduct.pop_back();
		}
		else if(SellTaskRobot.size()!=0)
		{
			if (CountDistanceWbRot(SellTaskRobot[TempISTask[i].OrderRobot]) == true)
			{
				SellTaskRobot[TempISTask[i].OrderRobot].sellOk = true;
				StatusNum SatTemp;
				SatTemp.ID = TempISTask[i].OrderRobot;
				SatTemp.BuyOk = false;
				SatTemp.SellOK = true;
				ID_BSRot.push_back(SatTemp);
				SellTaskRobot.erase(TempISTask[i].OrderRobot);//删除执行卖操作的指令
				TaskRobot.erase(TempISTask[i].OrderRobot); //获取有任务分配好的 TaskRobot 
				NewDetRot.push_back(TempISTask[i].OrderRobot);	
			}
		}
	}
	if(ID_BSRot.size()>1)
		sort(ID_BSRot.begin(), ID_BSRot.end(), CmpStatus);//
	return ID_BSRot;
}


void pid::PID_init() {
	//printf("PID_init begin \n");
	this->SetSpeed = 0.0;
	this->ActualSpeed = 0.0;
	this->err = 0.0;
	this->err_last = 0.0;
	this->voltage = 0.0;
	this->integral = 0.0;
	this->Kp = 0.2;
	this->Ki = 0.04; //之前KI值是0.015比之前的大
	this->Kd = 0.2;  //初始化过程
	//printf("PID_init end \n");
}

float pid::PID_realize(float speed,const double _num,float nowAngle) {
	int index;
	this->SetSpeed = speed;//目标方位
	this->ActualSpeed = nowAngle;//当前实际方位
	this->err = this->SetSpeed - this->ActualSpeed;
	this->integral += this->err;
	if (abs(this->err) >_num)
	{
		index = 0;
	}
	else

	{
		index = 1;
		//this->integral += this->err;
	}
	this->voltage = this->Kp*this->err + index * this->Ki*this->integral + this->Kd*(this->err - this->err_last); //可以用角速度代替   //算法具体实现过程
	this->err_last = this->err;
	this->ActualSpeed = this->voltage*1.0;
	// cout<<
	return this->ActualSpeed;//返回的是下一时刻的方位角

}
