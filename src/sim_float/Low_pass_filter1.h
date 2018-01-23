#ifndef LOW_PASS_FILTER1_H
#define LOW_PASS_FILTER1_H


class Low_Pass_Filter1
{
    public:
        Low_Pass_Filter1();
        virtual ~Low_Pass_Filter1();
        void Set_Parameters(double sampling_time, double time_constant);
        void Set_Init_Value(double init_value);
        double Get_Sampling_time(void);
        double Get_Time_Constant(void);
        double Calculate(double input);
        double Get_Value(void);
    protected:
    private:
        double output_value;
        double sampling_time;
        double time_constant;
};

#endif // LOW_PASS_FILTER1_H
