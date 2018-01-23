#include <iostream>
#include <cmath>
#include <fstream>
#include <windows.h>

#include "test.h"
#include "Integrator.h"
#include "Low_pass_filter1.h"
#include "pmsm.h"
#include "inverter.h"
#include "svpwm.h"
#include "adc.h"

using namespace std;

Test::Test()
{
    //ctor
}

Test::~Test()
{
    //dtor
}


void Test::Integrator_Test(void)
{
    int i;

    Integrator in0;

    cout << "Integrator test\n";
    in0.Set_Sampling_Time(1e-3);
    cout << "sample time=" << in0.Get_Sampling_time() << endl;

    for(i=1;i<10;i++)
    {
        in0.Calculate(1.);
        cout << "i=" << i << " intg=" << in0.Get_Value() << endl;
    }

}

void Test::LPF1_Test(void)
{
    int i;
    Low_Pass_Filter1 lpf;
    ofstream dfile;

    dfile.open("lpf.dat");

    cout << "lpf1 test" << endl;
    lpf.Set_Parameters(0.001,0.01);
    cout << "sample time=" << lpf.Get_Sampling_time() << endl;
    cout << "time constant=" << lpf.Get_Time_Constant() << endl;


    for(i=1;i<=50;i++)
    {
        lpf.Calculate(1.);
        cout << "i=" << i << " lpf=" << lpf.Get_Value() << endl;
        dfile << i << " " << lpf.Get_Value() << endl;
    }

    dfile.close();
}


void Test::Pmsm_Test(void)
{
    int32_t i;
    Pmsm m;
    double usa, usb;
    double time;
    double Ts=100e-6;
    double freq=4.;
    double angle;
    double u0=10;


    cout << "test pmsm" << endl;

    m.Set_Sampling_Time(Ts);
    m.Set_Parameters(0.1,0.001,0.001,0.1,2,0.001,0.01);

    for(i=1;i<=20000;i++)
    {
        time=i*Ts;
        angle=2*M_PI*freq*time;
        usa=u0*cos(angle); usb=u0*sin(angle);

        m.Calculate(usa,usb, 0.);
    }
    cout << "\nPMSM test finished" << endl;
}

Inverter inv;
void Test::Inverter_Test(void)
{

    double Ts=50e-6;
    double Rs=0.1;
    double Ls=30e-6;
    int32_t i;
    double time;
    uint16_t pwm_divisions=2400;
    //static times_in_t times_in;
    bemf_t bemf;
    //inv_measurement_t meas;
    complex16_t duty;
    uint16_t duty_mag;
    double ang;
    ofstream ofile;
    //static svpwm_t sv;


    ofile.open("inverter_slow.dat");

    duty_mag=500;
    foc_data.svm_length=duty_mag;

    cout << "inverter test" << endl;
    inv.Set_Sampling_Time(Ts, pwm_divisions);
    inv.Set_Parameters(Rs, Rs, Rs, Ls, Ls, Ls);

    init_svpwm(&sv, pwm_divisions);
    init_adc_variables();

    for(i=1;i<=350;i++)
    {
        int16_t iu_fix,iv_fix,iw_fix;

        time=i*Ts;
        ang=time*40.*10+0.0;


        duty.re=duty_mag*cos(ang);
        duty.im=duty_mag*sin(ang);

#if 0
        svpwm_sscm(&sv, &duty);

        inv.times_in.tu[0]=sv.t1up;
        inv.times_in.tu[1]=sv.t2up;
        inv.times_in.tu[2]=sv.t3up;
        inv.times_in.td[2]=sv.t3down;
        inv.times_in.td[1]=sv.t2down;
        inv.times_in.td[0]=sv.t1down;

        inv.times_in.m[0]=sv.m1;
        inv.times_in.m[1]=sv.m2;
#else
        foc_svm_improved ( &foc_data, duty.re, duty.im, 1 );

        inv.times_in.tu[0]=foc_data.svm_t0_up;
        inv.times_in.tu[1]=foc_data.svm_t1_up;
        inv.times_in.tu[2]=foc_data.svm_t2_up;
        inv.times_in.td[2]=foc_data.svm_t2_down;
        inv.times_in.td[1]=foc_data.svm_t1_down;
        inv.times_in.td[0]=foc_data.svm_t0_down;

        inv.times_in.m[0]=foc_data.svm_i1_samp_time_up;
        inv.times_in.m[1]=foc_data.svm_i2_samp_time_down;
#endif


#define BEMF_K 0.0002
        bemf.e1=BEMF_K*duty.re;
        bemf.e2=BEMF_K*(-duty.re/2+sqrt(3)/2*duty.im);
        bemf.e3=BEMF_K*(-duty.re/2-sqrt(3)/2*duty.im);

        inv.Calculate_PWM_Cycle(&bemf);

#define IOFFSET 2048
#define KMEAS 10
        adc.i1_digits=IOFFSET + KMEAS*inv.meas.m[0];
        adc.i2_digits=IOFFSET + KMEAS*inv.meas.m[1];

#if 0
        CalculateCurrentsSingleShunt();
#else
        mc_svm_extract_phase_currents();
#endif
        //adc.iu.adc_digits=adc.i1_digits;
        //calculate_adc_quantity(&adc.iu);
        //calculate_adc_quantity(&adc.iv);

#if 0
        iu_fix=((int32_t)adc.iu.value*adc.iu.gain)>>10;
        iv_fix=((int32_t)adc.iv.value*adc.iv.gain)>>10;
        iw_fix=((int32_t)adc.iw.value*adc.iu.gain)>>10;
#else
        iu_fix=adc.iu.value;
        iv_fix=adc.iv.value;
        iw_fix=adc.iw.value;
#endif

        //iu=iu_fix*In/32767;

        printf("%+6d %+6d %+6d \n",adc.iu.value,adc.iv.value,adc.iw.value);
        ofile << inv.Get_Time() << " " << foc_data.svm_stored_sector << " " ;               //1-2
        ofile << inv.times_in.m[0] << " " << inv.times_in.m[1] << " " << sv.m_udc << " ";   //3-5
        ofile << inv.times_in.tu[0] << " " << inv.times_in.td[0] << " ";              //6-7
        ofile << inv.times_in.tu[1] << " " << inv.times_in.td[1] << " ";        //8-9
        ofile << inv.times_in.tu[2] << " " << inv.times_in.td[2] << " ";        //10-11
       // ofile << adc.iu.value << " " << adc.iv.value << " " << adc.iw.value <<" "; //12-14
        ofile << iu_fix << " " << iv_fix << " " << iw_fix <<" "; //12-14
        ofile << ang << " " << duty.re << " " << duty.im <<" "<< time <<" "<< foc_data.svm_phi <<" "<< foc_data.svm_length << "\n";  //15-19


    }
    cout << "\nInverter test finished" << endl;

    ofile.close();
    system("run_octave_script.bat");
}
