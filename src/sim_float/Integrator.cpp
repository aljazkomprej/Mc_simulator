#include "Integrator.h"

Integrator::Integrator()
{
    //ctor
    this->integrator_value=0;
    this->sampling_time=1;
}

Integrator::~Integrator()
{
    //dtor
}

void Integrator::Set_Sampling_Time(double sampling_time)
{
    this->sampling_time=sampling_time;
}

double Integrator::Get_Sampling_time(void)
{
    return this->sampling_time;
}

void Integrator::Set_Init_Value(double init_value)
{
    this->integrator_value=init_value;
}

double Integrator::Calculate(double input)
{
    this->integrator_value += this->sampling_time*input;
    return this->integrator_value;
}

double Integrator::Get_Value(void)
{
    return this->integrator_value;
}
