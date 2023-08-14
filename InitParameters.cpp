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
ArbiterData ArbFrame;//ÿ�δ��������ݷ��ڴ˴�
bool InitData = false;//�����ж��Ƿ��ʼ����ȡ������ InitData: false δ��ʼ���� true ��ʼ�����
int WbDataNum = 0;
vector<int>NewAddRot;
vector<int>NewDetRot;

vector<WbSN>WBData_list;//����̨�������ݵļ���
vector<WorkbenchData>Robot_list;//�������������ݼ���

map<int, RobotSN>FreeRobot;//key ��ָ������ID RobotSN ��ָ������״̬����
map<int, RobotSN>TaskRobot;//����������� ȥִ��������������� 
map<int, RobotSN>BuyTaskRobot;//ִ��������Ļ����˼���
map<int, RobotSN>SellTaskRobot;//ִ���������Ļ����˼���
vector<WbSN> WbProductFull;//�洢����̨��Ʒ�Ѿ������õ��б�
vector<WbSN>WbNeedProduct;
vector<RobotSN> RobotFreeList;
vector<pair<WbSN, RobotSN>>StageWbRob;//���������빤��̨һһ��Ӧ�����˶�

vector<RefRobotSN>RefRobotList;//ȫ�ֱ��� ������

string toBinary(int n)//ʮ������ת��Ϊ������״̬
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

bool readUntilOK(int _frameID)//��ȡ֡�������Ƿ����ʵ��
{
	if (!InitData)
	{
		char line[1024];
		int count_col = 0;//   �����ж��ǵڼ�������
		int count_num_wb = -1;//����̨���
		int count_num_robot = -1;//���������
		while (fgets(line, sizeof line, stdin))
		{
			f1 << line << endl;
			if (line[0] == 'O' && line[1] == 'K')//���һ�����ݱ�־λ ����
			{
				WbDataNum = WBData_list.size();//��ȡ����̨����
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
					tempRefRob.robotPID.PID_init(); //��PIDִ�г�ʼ��
					RefRobotList.push_back(tempRefRob);//��ȡȫ�ֱ���
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
		//����ctrl+Z����������ѭ��
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
		//cout << "��ȡ����!" << endl;
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
				TempWb.disWbRobot.push_back(0);//��ʼ��4��0 �������ڽ��м������
			for (int i = 0; i < 4; i++)
				TempWb.Weight.push_back(0);
			TempWb.ID = countNUmWb;
			ArbFrame._wbsn[countNUmWb] = TempWb;
			countNUmWb++;
		}
		int countNumRobot = 0;
		/*
	int Wb_ID;//��ǰ����̨��IDֵ  -1����ʾ��ǰû�д����κι���̨����  [0,����̨����-1] ����ʾĳ����̨���±꣬�� 0 ��ʼ��
	//������˳�򶨡���ǰ�����˵����й��򡢳�����Ϊ����Ըù���̨���С�
	int OwnWuping_ID;// Я����Ʒ����  0 ��ʾδЯ����Ʒ 1 - 7 ��ʾ��Ӧ��Ʒ��
	double  TimeCostCoe;//ʱ���ֵϵ�� 0.8-1.0 ��Я����ƷʱΪ0
	double CrashCostCoe;//��ײ��ֵϵ��
	double AngleSpeed;//���ٶ�
	double x_Speed,y_Speed;//��ά���ٶ�
	double TowardAngle;//�˶��ĳ���Ƕ�
	double X_station;//
	double Y_station;
		*/
		for (int i = Flag_data_num_wb; i < data.size(); i += 10)
		{
			RobotSN TempRobot;
			TempRobot.OrderRobot = countNumRobot;//��ȡ���������
			TempRobot.Wb_ID = data[i];
			TempRobot.OwnWuping_ID = data[i + 1];
			TempRobot.TimeCostCoe = data[i + 2];
			TempRobot.CrashCostCoe = data[i + 3];
			TempRobot.AngleSpeed = data[i + 4];
			TempRobot.x_Speed = data[i + 5];
			TempRobot.y_Speed = data[i + 6];
			TempRobot.DSpeed = sqrt(TempRobot.x_Speed*TempRobot.x_Speed + TempRobot.y_Speed*TempRobot.y_Speed);//��ȡ�õ����ٶȴ�С
			TempRobot.TowardAngle = data[i + 7];
			TempRobot.X_station = data[i + 8];
			TempRobot.Y_station = data[i + 9];
			TempRobot.buyOK = false;
			TempRobot.sellOk = false;
			TempRobot.destroy = false;//�������������Ż����
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

bool cmp(WbSN c1, WbSN c2)// ����num�Ӵ�С���� �������㴦��
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
	return c1.second.OrderRobot < c2.second.OrderRobot;//ִ�д�С��������ʽ
}


vector<WbSN>FindWbIsFull(vector<int> _NoNeedBuy)//������ѯ����̨��״̬
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
					ArbFrame._wbsn[i].Weight[t] = 0.2*ArbFrame._wbsn[i].num +0.8* (7.0 - tempangle / 10.0); //����Ȩ�ط��䷽��
				}
				_WbProductFull.push_back(ArbFrame._wbsn[i]);//����Ʒ��״̬���Ĺ���̨ɸѡ���������ں�����������
			}
		}
	}
	//sort(_WbProductFull.begin(), _WbProductFull.end(), cmp);//�Ȱ���num�Ӵ�С�������� 
	return _WbProductFull;
}


vector<WbSN>FindWbNeedRawMat()//���ҹ���̨��Ҫ����ԭ����
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
				int temp = ArbFrame._wbsn[i].RawMaterialStatus ^ 6;//������
				if (temp != 0)
				{
					vector<int>NeedIdRaw = Ten2Two(temp);//NeedIdRaw ��Ϊ����Ҫ��ԭ������
					ArbFrame._wbsn[i].NowFrameNeedMat = NeedIdRaw;
					TempNeed.push_back(ArbFrame._wbsn[i]);
				}
				//if(ArbFrame._wbsn[i].RawMaterial.size()!=2)
				break;
			}
			case 5:
			{
				int temp = ArbFrame._wbsn[i].RawMaterialStatus ^ 10;//������
				if (temp != 0)
				{
					vector<int>NeedIdRaw = Ten2Two(temp);//NeedIdRaw ��Ϊ����Ҫ��ԭ������
					ArbFrame._wbsn[i].NowFrameNeedMat = NeedIdRaw;
					TempNeed.push_back(ArbFrame._wbsn[i]);

				}
				break;
			}
			case 6:
			{
				int temp = ArbFrame._wbsn[i].RawMaterialStatus ^ 12;//������
				if (temp != 0)
				{
					vector<int>NeedIdRaw = Ten2Two(temp);//NeedIdRaw ��Ϊ����Ҫ��ԭ������
					ArbFrame._wbsn[i].NowFrameNeedMat = NeedIdRaw;
					TempNeed.push_back(ArbFrame._wbsn[i]);
				}
				break;
			}
			case 7:
			{
				int temp = ArbFrame._wbsn[i].RawMaterialStatus ^ 112;//������
				if (temp != 0)
				{
					vector<int>NeedIdRaw = Ten2Two(temp);//NeedIdRaw ��Ϊ����Ҫ��ԭ������
					ArbFrame._wbsn[i].NowFrameNeedMat = NeedIdRaw;
					TempNeed.push_back(ArbFrame._wbsn[i]);
				}
				break;
			}
			case 8:
			{
				int temp = ArbFrame._wbsn[i].RawMaterialStatus ^ 128;//������
				if (temp != 0)
				{
					vector<int>NeedIdRaw = Ten2Two(temp);//NeedIdRaw ��Ϊ����Ҫ��ԭ������
					ArbFrame._wbsn[i].NowFrameNeedMat = NeedIdRaw;
					TempNeed.push_back(ArbFrame._wbsn[i]);
				}
				break;
			}
			default:
			{
				int temp = ArbFrame._wbsn[i].RawMaterialStatus ^ 240;//������
				if (temp != 0)
				{
					vector<int>NeedIdRaw = Ten2Two(temp);//NeedIdRaw ��Ϊ����Ҫ��ԭ������
					ArbFrame._wbsn[i].NowFrameNeedMat = NeedIdRaw;
					TempNeed.push_back(ArbFrame._wbsn[i]);
				}
			}
		}
			//��ȡ����̨ID �Լ��䵱ǰ״̬�����ԭ���ϣ��жϳ�����ǰ��Ҫ���ϵĹ���̨ �Լ���Ҫʲôԭ���� 
		}
	}
	return TempNeed;//��Ҫ��ԭ������
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
		return c1.ID < c2.ID;//������ΪʲôС���������Զ��������� Ӧ�����붼�������
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

RefRobotSN MatchSellWb(vector<WbSN> _WbNeedProduct, RefRobotSN _robot,bool& _flag, vector<int>&_BuySellOkList)//ȥ������������
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
	}//ƥ��ɹ�
	 //_robot.RenWuDivideOK = true;
	//SellTaskRobot[_robot.OrderRobot] = _robot;
	return _robot;
}



RobotSN MatchSellWb(vector<WbSN> _WbNeedProduct, RobotSN _robot)//ȥ������������
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
		_robot.MuBiaoWb = _WbNeedProduct[0];//ƥ��ɹ�
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


//���л��������񻮷�
vector<StatusNum> DivideRobotTask()//���Է���һ��buy sell����ָ��ļ���
{
	vector<RobotSN>TempISTask;//�����ж�������Ļ�����
	vector<RobotSN>TempISNoTask;//�����ж���û������Ļ�����
	vector<StatusNum>ID_BSRot;//�����˼��ϵ�ID
	for (int i = 0; i < 4; i++)
	{
		if (ArbFrame._robotsn[i].OwnWuping_ID == 0)//��ʾ�˻�����δ����
		{
			TempISNoTask.push_back(ArbFrame._robotsn[i]);
		}
		else
			TempISTask.push_back(ArbFrame._robotsn[i]);
	}
	for (int i = 0; i < TempISNoTask.size(); i++)
		{
			if (BuyTaskRobot.find(TempISNoTask[i].OrderRobot) == BuyTaskRobot.end()&& WbProductFull.size() != 0)//�Ȳ���������
			{
				//TempISNoTask[i].RenWuDivideOK = true;
				RobotSN temp = MatchBuyWbRobot(WbProductFull, TempISNoTask[i]);//����ƥ�乤��̨ ʵ�����ϵƥ��
				BuyTaskRobot[TempISNoTask[i].OrderRobot] = temp;//��������� 
				TaskRobot[TempISNoTask[i].OrderRobot] = temp;
				NewAddRot.push_back(TempISNoTask[i].OrderRobot);
				vector<WbSN>::iterator k = WbProductFull.begin();
				WbProductFull.erase(k);
			}
			else if(BuyTaskRobot.size()!=0)
			{
				//�鿴����������Ƿ�Ҫ����� ͨ����־λ��Ӧ����
				//Temp
				if (CountDistanceWbRot(BuyTaskRobot[TempISNoTask[i].OrderRobot]) == true)
				{
					BuyTaskRobot[TempISNoTask[i].OrderRobot].buyOK = true;//��ʾ�˻������ڵ�ǰ֡����ִ����ָ�����
					StatusNum SatTemp;
					SatTemp.ID = TempISNoTask[i].OrderRobot;
					SatTemp.BuyOk = true;
					SatTemp.SellOK = false;
					ID_BSRot.push_back(SatTemp);
					BuyTaskRobot.erase(TempISNoTask[i].OrderRobot);//��buyָ��� ������������ɾ��
					//û���������ж� ��������
					TaskRobot.erase(TempISNoTask[i].OrderRobot);
					NewDetRot.push_back(TempISNoTask[i].OrderRobot);
					//���ڴ�ִ��ɾ������ ����Ǽ��� ��ǰ������ID�� 
				}
				//�鿴֡�� ���д��� 
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
				SellTaskRobot.erase(TempISTask[i].OrderRobot);//ɾ��ִ����������ָ��
				TaskRobot.erase(TempISTask[i].OrderRobot); //��ȡ���������õ� TaskRobot 
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
	this->Ki = 0.04; //֮ǰKIֵ��0.015��֮ǰ�Ĵ�
	this->Kd = 0.2;  //��ʼ������
	//printf("PID_init end \n");
}

float pid::PID_realize(float speed,const double _num,float nowAngle) {
	int index;
	this->SetSpeed = speed;//Ŀ�귽λ
	this->ActualSpeed = nowAngle;//��ǰʵ�ʷ�λ
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
	this->voltage = this->Kp*this->err + index * this->Ki*this->integral + this->Kd*(this->err - this->err_last); //�����ý��ٶȴ���   //�㷨����ʵ�ֹ���
	this->err_last = this->err;
	this->ActualSpeed = this->voltage*1.0;
	// cout<<
	return this->ActualSpeed;//���ص�����һʱ�̵ķ�λ��

}
