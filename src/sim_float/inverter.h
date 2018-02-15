#ifndef INVERTER_H
#define INVERTER_H

#include <iostream>
#include <stdint.h>
#include <fstream>

// input data for PWM unit
typedef struct
{
    uint16_t tu[3]; // turn on time for particular legf
    uint16_t td[3];    // leg 1 on & off
    uint16_t m[2];  // measuring points
} times_in_t;

// single shunt measurement values
typedef struct
{
    double m[2];  // value of the measurement 1
} inv_measurement_t;

// back EMF structure
typedef struct
{
    double e1;
    double e2;
    double e3;
} bemf_t;

class Inverter
{
    public:
        Inverter();
        virtual ~Inverter();
        void Set_Sampling_Time(double pwm_period, uint16_t divisions);
        void Set_Parameters(double Rs1, double Rs2, double Rs3, double Ls1, double Ls2, double Ls3);
        double Get_Sampling_time(void);
        void Calculate_PWM_Cycle(const bemf_t *bemf);
        void Calculate(double u1, double u2, double u3, double e1, double e2, double e3);
        double GetMeasurementIDC(void);
        double Get_i1(void);
        double Get_i2(void);
        double Get_i3(void);
        double Get_Time(void);
        inv_measurement_t meas;
        times_in_t times_in;
        double uavg[3];     // average value of the voltage
        double iavg[3];     // average value of the voltage
        uint8_t CNT_mode;   //CNT mode (sawtooth 00, inverted sawtooth 01, triangle 10, inverted triangle 11;
        uint8_t middle_rl;  //middle reload (in case of triangle CNT mode) if 0 reload event at start event of PWM, if 1 reload event at start and at middle

    protected:
    private:
        void Calculate_Derivatives(void);

        // parameters
        double sampling_time;
        double Rs1, Rs2, Rs3;   // resistances
        double Ls1, Ls2, Ls3;   // inductances

        // state variables
        double i[3];         // phase currents

        // voltages
        double u1, u2, u3;
        double udc;

        // derivatives of state variables
        double d_i1, d_i2, d_i3;         // derivatives of stator currents

        double time;
        uint16_t divisions;
        uint16_t step;      // step inside pwm cycle
        uint16_t CNT;      // step inside pwm cycle
        uint16_t ss[3];    // switching state






        // data file
        std::ofstream dfile;

        //time
        int32_t no_of_calculations;
};

#endif // INVERTER_H
