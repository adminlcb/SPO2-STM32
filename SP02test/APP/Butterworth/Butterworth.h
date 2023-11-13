#ifndef  BUTTER_H
#define  BUTTER_H

//函数声明

typedef struct
{
    float a;
	float in_old;
    float out;
	char type;
} fof_Type;

extern fof_Type fof_1;
extern fof_Type fof_2;
void fof_Init(fof_Type *fof, float f, float delta_ms, char HPForLPF);	//f为截止频率，delta_ms为时间间隔，HPForLPF为类型选择
float FirstOrderFilter(fof_Type *fof, float in);


#endif


