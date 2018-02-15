#ifndef TEST_H
#define TEST_H

#include "Integrator.h"

#define OEmod 1U //Online Engineering modulator
#define ELMOSmod 0U //ELMOS modulator

class Test
{
    public:
        Test();
        virtual ~Test();
        void Integrator_Test(void);
        void LPF1_Test(void);
        void Pmsm_Test(void);
        void Inverter_Test(void);
    protected:
    private:
};


#endif // TEST_H
