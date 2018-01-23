#include <iostream>
#include <fstream>
#include <cmath>
#include <stdint.h>

#include "pmsm.h"

Pmsm::Pmsm()
{
    dfile.open("pmsm.dat");

    //dfile << "file opened" << std::endl;

    isd=0;
    isq=0;
    wm=0;
    we=0;
    mech_angle=0;
    electrical_angle=0;
    torque=0;
    isa=0;
    isb=0;
    no_of_calculations=0;

}

Pmsm::~Pmsm()
{
    //dtor
    dfile.close();
}

void Pmsm::Set_Sampling_Time(double sampling_time)
{
    this->sampling_time=sampling_time;
}

double Pmsm::Get_Sampling_time(void)
{
    return this->sampling_time;
}

void Pmsm::Set_Parameters(double Rs, double Ld, double Lq, double Psimd, int16_t p, double J, double B)
{
    this->Rs=Rs;
    this->Ld=Ld;
    this->Lq=Lq;
    this->Psimd=Psimd;
    this->p=p;
    this->J=J;
    this->B=B;
}

void Pmsm::Calculate(double usa, double usb, double torque_load)
{
    double msin,mcos;

    this->usa=usa;
    this->usb=usb;
    this->torque_load=torque_load;

    no_of_calculations++;

    msin=sin(electrical_angle);
    mcos=cos(electrical_angle);

    // transform voltages from ab to dq
    usd= mcos*usa + msin*usb;
    usq=-msin*usa + mcos*usb;

    // calculate current derivatives
    d_isd=(usd-Rs*isd+we*Lq*isq)/Ld;
    d_isq=(usq-Rs*isq-we*(Ld*isd+Psimd))/Lq;

    // integrate stator current
    isd+=d_isd*sampling_time;
    isq+=d_isq*sampling_time;

    // calculate torque
    torque=3./2.*p*(Psimd+(Ld-Lq)*isd)*isq;

    // calculate speed derivative
    d_wm=(torque-torque_load-B*wm)/J;

    //integrate mechanical speed
    wm+=d_wm*sampling_time;

    //calculate mechanical angle
    mech_angle+=wm*sampling_time;
    if(mech_angle>2*M_PI)
    {
        mech_angle -= 2*M_PI;
    }
    else if(mech_angle<-2*M_PI)
    {
        mech_angle += 2*M_PI;
    }

    //calculate electrical speed
    we=wm*p;

    electrical_angle=mech_angle*p;

    //transform stator current from dq to ab
    isa=mcos*isd - msin*isq;
    isb=msin*isd + mcos*isq;

    // dodaj avtomatsko shranjevanje v datoteko na vsakem koraku
    dfile << no_of_calculations*sampling_time << " ";
    dfile << isd << " " << " " << isq << " ";
    dfile << usd << " " << " " << usq << " ";
    dfile << isa << " " << " " << isb << " ";
    dfile << usa << " " << " " << usb << " ";
    dfile << torque << " " << " " << we << " ";
    dfile << mech_angle << " " << " " << electrical_angle << " ";
    dfile << wm << " ";

    dfile  <<  std::endl;
}


void Pmsm::Calculate_Derivatives(void)
{
    d_isd=(usd-Rs*isd+we*Lq*isq)/Ld;
    d_isq=(usq-Rs*isq-we*(Ld*isd+Psimd))/Lq;
    d_wm=(torque-torque_load-B*wm)/J;
    d_mech_angle=wm;
}

void Pmsm::Calculate_Torque(void)
{
    torque=3/(2*p)*(Psimd+(Ld-Lq)*isd)*isq;
}


double Pmsm::Get_wm(void)
{
    return this->wm;
}

double Pmsm::Get_isa(void)
{
    return this->isa;
}

double Pmsm::Get_isb(void)
{
    return this->isb;
}
