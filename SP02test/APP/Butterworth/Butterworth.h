#ifndef  BUTTER_H
#define  BUTTER_H

//��������

typedef struct
{
    float a;
	float in_old;
    float out;
	char type;
} fof_Type;

extern fof_Type fof_1;
extern fof_Type fof_2;
void fof_Init(fof_Type *fof, float f, float delta_ms, char HPForLPF);	//fΪ��ֹƵ�ʣ�delta_msΪʱ������HPForLPFΪ����ѡ��
float FirstOrderFilter(fof_Type *fof, float in);


#endif


