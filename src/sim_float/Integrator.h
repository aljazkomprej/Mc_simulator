#ifndef INTEGRATOR_H
#define INTEGRATOR_H


class Integrator
{
    public:
        Integrator();
        virtual ~Integrator();
        void Set_Sampling_Time(double sampling_time);
        void Set_Init_Value(double init_value);
        double Get_Sampling_time(void);
        double Calculate(double input);
        double Get_Value(void);
    protected:
    private:
        double integrator_value;
        double sampling_time;
};

#endif // INTEGRATOR_H
