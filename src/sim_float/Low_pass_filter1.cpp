#include "Low_pass_filter1.h"

Low_Pass_Filter1::Low_Pass_Filter1()
{
    //ctor
    this->output_value=0;
    this->sampling_time=1;
}

Low_Pass_Filter1::~Low_Pass_Filter1()
{
    //dtor
}

void Low_Pass_Filter1::Set_Parameters(double sampling_time, double time_constant)
{
    this->sampling_time=sampling_time;
    this->time_constant=time_constant;
}

double Low_Pass_Filter1::Get_Sampling_time(void)
{
    return this->sampling_time;
}

double Low_Pass_Filter1::Get_Time_Constant(void)
{
    return this->time_constant;
}

void Low_Pass_Filter1::Set_Init_Value(double init_value)
{
    this->output_value=init_value;
}

double Low_Pass_Filter1::Calculate(double input)
{
    double k;
    k=this->sampling_time / this->time_constant;
    this->output_value = (this->output_value)*(1. - k) + k * input;
    return this->output_value;
}

double Low_Pass_Filter1::Get_Value(void)
{
    return this->output_value;
}
