#pragma once
#include<math.h>
class HamThuoc
{
public:
	double hlimitleft;
	double hlimitright;
	virtual double GetValue(double in) = 0;
};

class hamtamgiac : public HamThuoc
{
public:
	hamtamgiac(double left, double mid, double right);
	hamtamgiac();
	~hamtamgiac();
	double GetValue(double in);
private:
	double limitleft;
	double limitright;
	double pchange;
};

class hamhinhthangtrai : public HamThuoc
{
public:
	hamhinhthangtrai(double mid, double right);
	hamhinhthangtrai();
	~hamhinhthangtrai();
	double GetValue(double in);
private:
	double limitright;
	double pchange;
};

class hamhinhthangphai : public HamThuoc
{
public:
	hamhinhthangphai(double left, double mid);
	hamhinhthangphai();
	~hamhinhthangphai();
	double GetValue(double in);
private:
	double limitleft;
	double pchange;
};
//-----------------------------------------------------//
class ThuocTinh
{
public:
	int sohamthuoc;
	ThuocTinh(int soham, HamThuoc** Hamthuoc);
	ThuocTinh();
	~ThuocTinh();
	double* GetValue(double in);
	double* output;
	HamThuoc** ham;
private:

};
//----------------------------------------------------//
class MyFuzzy
{
public:
	ThuocTinh* thuoctinh;
	double* maxvalue;
	double quytaclaymin[4][3];
	MyFuzzy(ThuocTinh* thuoctinht, double* maxvalue);
	~MyFuzzy();
	double getvalue(double in1);

private:

};

//---------------------------------------------------------//
hamtamgiac::hamtamgiac(double left, double mid, double right)
{
	limitleft = left;
	pchange = mid;
	limitright = right;
	hlimitleft = left;
	hlimitright = right;
}
hamtamgiac::hamtamgiac(){}
hamtamgiac::~hamtamgiac()
{
}
double hamtamgiac::GetValue(double in){
	if (in >= limitleft&&in < pchange) return (in - limitleft) / (pchange - limitleft);
	else if (in >= pchange&&in < limitright) return (limitright - in) / (limitright - pchange);
	else return 0;
}
//---------------------------------------------------------//
hamhinhthangtrai::hamhinhthangtrai(double mid, double right)
{
	pchange = mid;
	limitright = right;
}
hamhinhthangtrai::hamhinhthangtrai()
{
}
hamhinhthangtrai::~hamhinhthangtrai()
{
}
double hamhinhthangtrai::GetValue(double in){
	if (in < pchange) return 1;
	else if (in >= pchange&&in < limitright) return (limitright - in) / (limitright - pchange);
	else return 0;
}
//-------------------------------------------------------//
hamhinhthangphai::hamhinhthangphai(double left, double mid)
{
	limitleft = left;
	pchange = mid;
}
hamhinhthangphai::hamhinhthangphai(){}
hamhinhthangphai::~hamhinhthangphai()
{
}
double hamhinhthangphai::GetValue(double in){
	if (in >= limitleft&&in < pchange) return (in - limitleft) / (pchange - limitleft);
	else if (in >= pchange) return 1;
	else return 0;
}

//--------------------------------------------------------------//
ThuocTinh::ThuocTinh(int soham, HamThuoc** Hamthuoc)
{
	sohamthuoc = soham;
	ham = Hamthuoc;
}

ThuocTinh::~ThuocTinh()
{
}
double* ThuocTinh::GetValue(double in){
	output = new double[sohamthuoc];
	for (int i = 0; i < sohamthuoc; i++){
		output[i] = ham[i]->GetValue(in);
	}
	return output;
}
//-------------------------------------------------------------//
MyFuzzy::MyFuzzy(ThuocTinh* thuoctinht, double* maxvaluet)
{
	thuoctinh = thuoctinht;
	maxvalue = maxvaluet;
	/**quytaclaymin = new double[4];
	for (int i = 0; i < thuoctinh[0].sohamthuoc; i++){
	quytaclaymin[i] = new double[3];
	}*/
}

MyFuzzy::~MyFuzzy()
{
}
double MyFuzzy::getvalue(double in1){
	int sizett = thuoctinh[0].sohamthuoc;
	thuoctinh[0].GetValue(in1);
	double ketqua = 0;
	for (int i = 0; i < sizett; i++){
		ketqua += thuoctinh[0].output[i] * maxvalue[i];
	}
	return ketqua;
}
