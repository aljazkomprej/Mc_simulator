#include <iostream>
#include <fstream>
#include <cmath>
#include <stdint.h>

#include "inverter.h"

Inverter::Inverter()
{
    uint16_t j;

    dfile.open("inverter.dat");

    for(j=0;j<3;j++)
    {
        uavg[j]=0;
        iavg[j]=0;
        i[j]=0;
    }
    time=0;
    step=0;
    udc=12.;
    no_of_calculations=0;
}

Inverter::~Inverter()
{
    //dtor
    dfile.close();
}

void Inverter::Set_Sampling_Time(double pwm_period, uint16_t divisions)
{
    this->sampling_time=pwm_period/divisions;
    this->divisions=divisions;
}

double Inverter::Get_Sampling_time(void)
{
    return this->sampling_time;
}

void Inverter::Set_Parameters(double Rs1, double Rs2, double Rs3, double Ls1, double Ls2, double Ls3)
{
    this->Rs1=Rs1;
    this->Rs2=Rs2;
    this->Rs3=Rs3;
    this->Ls1=Ls1;
    this->Ls2=Ls2;
    this->Ls3=Ls3;
}



void Inverter::Calculate(double u1, double u2, double u3, double e1, double e2, double e3)
{
    double den;

    no_of_calculations++;

    den=Ls1*Ls2+Ls1*Ls3+Ls2*Ls3;


    // calculate current derivatives
    d_i1 = (-(Ls2+Ls3)*Rs1*i[0] + Ls3*Rs2*i[1] + Ls2*Rs3*i[2] - Ls2*u3-Ls3*u2+Ls3*u1+Ls2*u1+e2*Ls3-e1*Ls3+e3*Ls2-e1*Ls2)/den;
    d_i2 = (Ls3*Rs1*i[0] - (Ls1+Ls3)*Rs2*i[1] + Ls1*Rs3*i[2] - Ls1*u3+Ls3*u2+Ls1*u2-Ls3*u1-e2*Ls3+e1*Ls3+e3*Ls1-e2*Ls1)/den;
    d_i3 = (Ls2*Rs1*i[0] + Ls1*Rs2*i[1] - (Ls1+Ls2)*Rs3*i[2] + Ls2*u3+Ls1*u3-Ls1*u2-Ls2*u1-e3*Ls2+e1*Ls2-e3*Ls1+e2*Ls1)/den;

    // integrate stator current
    i[0] +=d_i1*sampling_time;
    i[1] +=d_i2*sampling_time;
    i[2] +=d_i3*sampling_time;

#if 1
    // dodaj avtomatsko shranjevanje v datoteko na vsakem koraku
    dfile << no_of_calculations*sampling_time << " ";     // 1
    dfile << i[0] << " " << i[1] << " " << i[2] << " " ;  // 2 - 4
    dfile << u1 << " " << u2 << " " << u3 << " " ;         // 5 - 7
    dfile << e1 << " " << e2 << " " << e3 << " " ;          // 8 - 10
    dfile << meas.m[0] << " " << meas.m[1] << " ";          // 11 - 12
    dfile << uavg[0] << " " << uavg[1] << " " << uavg[2] << " " ;   // 13 - 15
    dfile << iavg[0] << " " << iavg[1] << " " << iavg[2] << " " ;   // 16 - 18
    dfile  <<  std::endl;
#endif
}


// get value of the current in dc link
double Inverter::GetMeasurementIDC(void)
{
    uint16_t n;
    double idc;

    idc=0.;

    // from the switching state determine dc link current
    // if ss[n]==0 (bottom switch is on, add this partucular phase current to idc
    for(n=0;n<3;n++)
    {
        if( ss[n] == 0 ) idc += i[n];
    }

    return idc;
}



/*
Za izracun enega prekllopnega intervala
-ima vsaka veja po en vklop in izklop, ki je izracunan vnaprej
-inducirano napetost izracunavam znotraj preklopnega intervala za vsako tocko.

Zeleno je, da shranim:
-ali vse izracunane tocke
-ali pa omogocim decimacijo

Kot parametre podam:
-tocke preklopov posameznih faz
-inducirano napetost (ali naj uporabim interpolacijo? - ne )


*/

void Inverter::Calculate_PWM_Cycle(const bemf_t *bemf)
{
    uint16_t k,j;
    double u[3];
    double usum[3]; // for the calculation of the average voltage
    double isum[3]; // for the calculation of the average current


    for(j=0;j<3;j++)
    {
        usum[j]=0;
        isum[j]=0;
    }

    for(step=1; step < divisions; step++)
    {
        time += sampling_time;

        // calculate phase voltages
        for(k=0; k<3; k++)  // calculate phase voltage for each individual phase
        {
            //if(( step < times_in.tu[k] ) || (step >=times_in.td[k] ) ) //times_in.tu before times_in.td
            if(( step < times_in.tu[k] ) && (step > times_in.td[k] ) )  //times_in.td before times_in.tu
            {
                ss[k]=0;
            }
            else
            {
                ss[k]=1;
            }
            u[k]=ss[k]*udc;
        }

        Calculate(u[0],u[1],u[2],bemf->e1,bemf->e2,bemf->e3);

        for(j=0;j<3;j++)
        {
            usum[j] += u[j];
            isum[j] += i[j];
        }

        for(k=0; k<2; k++)
        {
            if( times_in.m[k] == step )
            {
                meas.m[k]=GetMeasurementIDC();
            }
        }
    }

    for(j=0;j<3;j++)
    {
        uavg[j]=usum[j]/divisions;
        iavg[j]=isum[j]/divisions;
    }

    return;
}




double Inverter::Get_i1(void)
{
    return this->i[0];
}

double Inverter::Get_i2(void)
{
    return this->i[1];
}

double Inverter::Get_i3(void)
{
    return this->i[2];
}

double Inverter::Get_Time(void)
{
    return time;
}
