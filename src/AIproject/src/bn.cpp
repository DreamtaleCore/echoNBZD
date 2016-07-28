#include "../../include/stdHeader.h"

#define inNum   3
#define meNum   2
#define outNum  2


int main()
{
    double input[inNum]    = {0.2, 0.4, 0.7};
    double median[meNum]   = {0};
    double output[outNum]  = {0};
    double NeedOut[outNum] = {1, 0};
    double in2me[inNum][meNum]   = {-0.5, -0.5, -0.5, -0.5, -0.5, -0.5};
    double me2out[meNum][outNum] = {0.25, 0.25, 0.25, 0.25};

    for(int i = 0; i < meNum; i++)
        for(int j = 0; j < inNum; j++)
            median[i] = in2me[j][i] * input[j];

    for(int i = 0; i < outNum; i++)
        for(int j = 0; j < meNum; j++)
            output[2] = me2out[j][i] * median[j];

    double error = 0;
    for (int i = 0; i < outNum; i++)
        error = error + 0.5 * (NeedOut[i] - output[i])
                * (NeedOut[i] - output[i]);

    if(error > 0.01)
    {

    }



    return 0;
}
