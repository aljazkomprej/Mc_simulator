#include <iostream>
#include <fstream>

#include "Integrator.h"
#include "Low_pass_filter1.h"
#include "pmsm.h"
#include "test.h"
//#include "test_fix_float_sim.h"


extern "C" void test_fix_float_sim(void);
void test_motor_control_wo_pwm(void);

//using namespace std;


int main(void)
{
    std::cout << "Motor control simulator " << std::endl;

    Test t;
    //t.Integrator_Test();9
    //t.LPF1_Test();
    //t.Pmsm_Test();


    //test_fix_float_sim();
    //test_motor_control_wo_pwm();
    t.Inverter_Test();

    return 0;
}
