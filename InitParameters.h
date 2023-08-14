#pragma once//���������ļ� 
#ifndef _InitParameters
#define _InitParameters
//#include"dwa.h"
#include<vector>
#include<map>
using namespace std;
typedef struct WorkbenchData {
	double Row_Coordinates;//������
	double Col_Coordinates;//������
	char num;//���
	int Order;//����̨���˳��
	int robotid;
	vector<double>disWbRobot;
}WBData;

typedef struct WbStatusNow {
	int num;
	double x;
	double y;
	int LeftFrame;//-1û�������� 0 ��ʾ������������������� �� >=0����ʾʣ������֡����  
	int RawMaterialStatus;//ԭ����״̬ ��ʾ��ǰ��ӵ�е���Ʒ ��ʮ�������ֱ�ʾ ʹ��ʱ��Ҫ����ת��Ϊ������ 
	vector<int>RawMaterial;//
	vector<int>NowFrameNeedMat;//��ǰ֡����Ҫ��ԭ����
	int ProductStatus;//0 ��ʾ�� 1 ��ʾ��
	vector<double> disWbRobot;//����������빺��̨�ľ���

	vector<double>Weight;//����Ȩ�ز���  ���ں��ڷ���

	int ID;

	int robotid;//Ŀ�������ID��

	int WbDestory ;//500֡���򲻵���Ʒ �����ٴ��� 

}WbSN;//����̨��ǰ״̬

typedef struct RobotStatusNow {
	int OrderRobot;//���������
	int Wb_ID;//��ǰ����̨��IDֵ  -1����ʾ��ǰû�д����κι���̨����  [0,����̨����-1] ����ʾĳ����̨���±꣬�� 0 ��ʼ��
	//������˳�򶨡���ǰ�����˵����й��򡢳�����Ϊ����Ըù���̨���С�
	int OwnWuping_ID;// Я����Ʒ����  0 ��ʾδЯ����Ʒ 1 - 7 ��ʾ��Ӧ��Ʒ��
	double TimeCostCoe;//ʱ���ֵϵ�� 0.8-1.0 ��Я����ƷʱΪ0
	double CrashCostCoe;//��ײ��ֵϵ��
	double AngleSpeed;//���ٶ�
	double x_Speed, y_Speed;//��ά���ٶ�
	double TowardAngle;//�˶��ĳ���Ƕ�
	double X_station;//
	double Y_station;

	//bool IsOrHuoWU;//
	bool buyOK;//���������־λ
	bool sellOk;//����������־λ
	bool destroy;//�ж��Ƿ�����Ʒ
	bool AngleSpeedSetOk;//�����жϽ��ٶ��Ƿ����ú�
	double DSpeed;//

	bool RenWuDivideOK;//�жϻ����������Ƿ����õı�־λ
	int BindRobotWb;//���������빤��̨���а� 
	WbSN MuBiaoWb;//��ʾ�����˵�Ŀ��λ��
}RobotSN;


class pid {
public:
	float SetSpeed; //�����趨ֵ
	float ActualSpeed; //����ʵ��ֵ
	float err; //����ƫ��ֵ
	float err_last; //������һ��ƫ��ֵ
	float Kp, Ki, Kd; //������������֡�΢��ϵ��
	float voltage; //�����ѹֵ������ִ�����ı�����
	float integral; //�������ֵ
public:
	void PID_init();
	float PID_realize(float speed, const double _num,float nowAngle);
};


typedef struct RobotRefresh {
	WbSN MuBiaoWb;//��ʾ�����˵�Ŀ��λ��
	double x_station;//�����˵�ǰ����
	double y_station;
	double rawangle;//�����˵�ǰ�Ƕ�
	double rawspeed;//�����˵�ǰ���ٶ�
	double speed;//��ǰ�������ٶ�


	int status;//-1 ���� 1 ȥ�� 2 ȥ�� 
	int ID;
	int owning_ID;
	bool MatchOk;
	int KJWbId;
	bool destoryOK;//�������𻵴���

	pid robotPID;



}RefRobotSN;//���»����˵ĵ�ǰ��Ϣ




typedef struct Arbiter {//ÿһ֡���������ݴ洢
	int FrameNum;//֡���
	int MoneySum;//��ǰ��Ǯ��
	int AllWbNum;//��ǰ���еĹ���̨��
	WbSN _wbsn[50];//���ֵ��ʼ��
	RobotSN _robotsn[4];//�����˸���״̬
}ArbiterData;//�洢ÿһ֡������

typedef struct StatusNum {// ��ǰ���ʱ ������״̬ 
	int ID;
	int num;
	bool SellOK;
	bool BuyOk;
	bool DestoryOK;
};



//typedef struct _pid {
//	float SetSpeed; //�����趨ֵ
//	float ActualSpeed; //����ʵ��ֵ
//	float err; //����ƫ��ֵ
//	float err_last; //������һ��ƫ��ֵ
//	float Kp, Ki, Kd; //������������֡�΢��ϵ��
//	float voltage; //�����ѹֵ������ִ�����ı�����
//	float integral; //�������ֵ
//
//	//void PID_init();
//
//	//float PID_realize(float speed);
//}pid;



//ÿ�ν�����ȡ��Ϣ  
/*������������������ݸ�ʽ����*/
string toBinary(int n);//ʮ������ת��Ϊ������״̬
vector<int> Ten2Two(int data);
bool readUntilOK(int _frameId);//��ȡ֡�������Ƿ����ʵ��
bool cmp(WbSN c1, WbSN c2);// ����num�Ӵ�С���� �������㴦��
bool cmp1(pair<WbSN, RobotSN> c1, pair<WbSN, RobotSN> c2);
vector<WbSN>FindWbIsFull(vector<int> _NoNeedBuy);//������ѯ����̨��״̬            ������˵�ǰ����̨�����õĲ���
vector<WbSN>FindWbNeedRawMat();//���ҹ���̨��Ҫ����ԭ����     ������˵�ǰ����̨����Ҫ�Ĳ��� 
//vector<WbSN>FindWbisNeed();//������ѯ����̨�Ƿ���Ҫ����
//vector<RobotSN>FindRobotIsFull();//������ѯ�����˵�״̬
vector<StatusNum> DivideRobotTask();//���Է���һ��buy sell����ָ��ļ���
bool CmpStatus(StatusNum c1, StatusNum c2);
//vector<DWA*>DwaWbRob;
//bool cmpRobotId(StatusNum c1, StatusNum c2);//�Ƚϻ�����ID��С�����մ�С�����˳��������
//void PrintStageWbRobot(int _FrameID, vector<DWA*>_DwaWbRob, vector<DWA*>_buy_ok);//������õ����ݴ�ӡ���
//vector<DWA*> FunDwaWbRob(vector<DWA*>&_DwaWbRob);//�������ˢ�º���
//bool CompareSameId(vector<DWA*>_DwaWbRob, DWA* _TempDwa);//�Ƚ��Ƿ����������������ͬ�����
//void/*vector<DWA*>*/ ResourceMatch(vector<WbSN> _WbProductFull, vector<WbSN>  _FindWbIsNeed, vector<RobotSN> _RobotFreeList, vector<DWA*>&DwaWbRob);
RefRobotSN MatchSellWb(vector<WbSN> _WbNeedProduct, RefRobotSN _robot,bool& _flag,vector<int>& _BuySellOkList);

bool CmpMatchXz(WbSN c1, WbSN c2);

#endif // _InitParameters

