#include "Butterworth.h"

/*
* 低通滤波器的实现
*/

fof_Type fof_1;
fof_Type fof_2;

//初始化 一阶rc滤波器
//滤波系数A等于 1/（1+（1/（2兀Tf）））
void fof_Init(fof_Type *fof, float f, float delta_ms, char HPForLPF) 
{
	fof->in_old = 0;
    fof->out = 0;
	fof->type = HPForLPF;
	fof->a = 2 * 3.141592653589793f * f * delta_ms / 1000.f;
	if(!HPForLPF)
	{
		fof->a = 1.f / fof->a;
	}
	fof->a = 1.f / (1.f + fof->a);
}

//一阶巴特沃斯
float FirstOrderFilter(fof_Type *fof, float in)
{
	if(fof->type)
	{
		fof->out = fof->a * fof->out + fof->a * (in - fof->in_old);
		fof->in_old = in;
	}
	else
		fof->out = fof->a * in + (1 - fof->a) * fof->out;
    return fof->out;
}
