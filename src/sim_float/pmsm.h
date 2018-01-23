#ifndef PMSM_H
#define PMSM_H

#include <stdint.h>
#include <fstream>

class Pmsm
{
    public:
        Pmsm();
        virtual ~Pmsm();
        void Set_Sampling_Time(double sampling_time);
        void Set_Parameters(double Rs, double Ld, double Lq, double Psimd, int16_t p, double J, double B);
        double Get_Sampling_time(void);
        void Calculate(double usa, double usb, double torque_load);
        double Get_wm(void);
        double Get_isa(void);
        double Get_isb(void);
    protected:
    private:
        void Calculate_Derivatives(void);
        void Calculate_Torque(void);

        // parameters
        double sampling_time;
        double Rs;
        double Ld;
        double Lq;
        double Psimd;
        int16_t p;          // polepairs
        double J;           // moment of inertia
        double B;           // friction constant

        // state variables
        double isd;         // direct current
        double isq;         // quadrature current
        double wm;          // mechanical speed
        double mech_angle;  // mechanical angle

        // derivatives of state variables
        double d_isd;         // derivative of direct current
        double d_isq;         // derivative of quadrature current
        double d_wm;          // derivative of mechanical speed
        double d_mech_angle;  // derivative of mechanical angle


        // calculated variables
        double we;          // electrical angular speed
        double electrical_angle;
        double torque;      // toruqe
        double psisd;
        double psisq;
        double usd;
        double usq;
        double isa;
        double isb;

        // input variables
        double usa;
        double usb;
        double torque_load;

        // data file
        std::ofstream dfile;

        //time
        int32_t no_of_calculations;
};

#endif // PMSM_H
